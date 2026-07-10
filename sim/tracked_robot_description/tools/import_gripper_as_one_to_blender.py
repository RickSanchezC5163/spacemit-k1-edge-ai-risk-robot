import math
import struct
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path

import bpy
from mathutils import Vector


PKG_DIR = Path(__file__).resolve().parents[1]
MESH_DIR = PKG_DIR / "meshes" / "gazebo_roscar"
OUT_PATH = PKG_DIR / "blender" / "tracked_robot_gripper_as_one.blend"
MERGED_STL = MESH_DIR / "gripper_visual_merged_ros_m.stl"
SCALE = 0.001


def reset_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0


def material(name, rgba):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = rgba
    return mat


def assign(obj, mat):
    if hasattr(obj.data, "materials"):
        obj.data.materials.append(mat)
    return obj


def collection(name):
    col = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(col)
    return col


def move_to_collection(obj, col):
    for current in list(obj.users_collection):
        current.objects.unlink(obj)
    col.objects.link(obj)


def stl_bounds(path):
    data = path.read_bytes()
    triangles = struct.unpack("<I", data[80:84])[0]
    mins = [float("inf")] * 3
    maxs = [float("-inf")] * 3
    offset = 84
    for _ in range(triangles):
        values = struct.unpack("<12f", data[offset : offset + 48])
        for vertex_index in range(3):
            xyz = values[3 + vertex_index * 3 : 6 + vertex_index * 3]
            for axis, value in enumerate(xyz):
                mins[axis] = min(mins[axis], value)
                maxs[axis] = max(maxs[axis], value)
        offset += 50
    return mins, maxs


def ros_from_sw_mm(sw_xyz, pivot_mm):
    sw_x, sw_y, sw_z = sw_xyz
    return (
        -(sw_y - pivot_mm[1]) * SCALE,
        (sw_x - pivot_mm[0]) * SCALE,
        (sw_z - pivot_mm[2]) * SCALE,
    )


def load_binary_stl_triangles(path, pivot_mm):
    data = path.read_bytes()
    triangles = struct.unpack("<I", data[80:84])[0]
    vertices = []
    faces = []
    offset = 84
    for _ in range(triangles):
        values = struct.unpack("<12f", data[offset : offset + 48])
        face = []
        for vertex_index in range(3):
            raw = values[3 + vertex_index * 3 : 6 + vertex_index * 3]
            face.append(len(vertices))
            vertices.append(ros_from_sw_mm(raw, pivot_mm))
        faces.append(tuple(face))
        offset += 50
    return vertices, faces


def parse_3mf_mesh(path):
    with zipfile.ZipFile(path) as archive:
        model_name = None
        for name in archive.namelist():
            if name.endswith("object_1.model"):
                model_name = name
                break
        if model_name is None:
            raise ValueError(f"{path.name} does not contain object_1.model")
        root = ET.fromstring(archive.read(model_name))
    ns = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}
    vertices = []
    for vertex in root.findall(".//m:vertices/m:vertex", ns):
        vertices.append(
            (
                float(vertex.attrib["x"]),
                float(vertex.attrib["y"]),
                float(vertex.attrib["z"]),
            )
        )
    faces = []
    for tri in root.findall(".//m:triangles/m:triangle", ns):
        faces.append((int(tri.attrib["v1"]), int(tri.attrib["v2"]), int(tri.attrib["v3"])))
    return vertices, faces


def create_3mf_manual_object(path, mat, col):
    verts_mm, faces = parse_3mf_mesh(path)
    center = Vector(
        (
            sum(v[0] for v in verts_mm) / len(verts_mm),
            sum(v[1] for v in verts_mm) / len(verts_mm),
            sum(v[2] for v in verts_mm) / len(verts_mm),
        )
    )
    verts = [((x - center.x) * SCALE, (y - center.y) * SCALE, (z - center.z) * SCALE) for x, y, z in verts_mm]
    mesh = bpy.data.meshes.new(f"{path.stem}_mesh")
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    obj = bpy.data.objects.new("manual_align_mechanical_claw_servo_bracket_3mf", mesh)
    obj.location = (0.0, -0.12, 0.045)
    col.objects.link(obj)
    assign(obj, mat)
    return obj


def write_binary_stl(path, name, vertices, faces):
    header = f"{name} generated in ROS meters".encode("ascii", "ignore")[:80]
    header = header + b" " * (80 - len(header))
    with path.open("wb") as out:
        out.write(header)
        out.write(struct.pack("<I", len(faces)))
        for face in faces:
            a = Vector(vertices[face[0]])
            b = Vector(vertices[face[1]])
            c = Vector(vertices[face[2]])
            normal = (b - a).cross(c - a)
            if normal.length > 0:
                normal.normalize()
            out.write(struct.pack("<3f", normal.x, normal.y, normal.z))
            for idx in face:
                x, y, z = vertices[idx]
                out.write(struct.pack("<3f", x, y, z))
            out.write(struct.pack("<H", 0))


def add_text(name, text, loc, mat):
    bpy.ops.object.text_add(location=loc, rotation=(math.radians(70), 0.0, 0.0))
    obj = bpy.context.object
    obj.name = name
    obj.data.body = text
    obj.data.align_x = "CENTER"
    obj.data.size = 0.008
    assign(obj, mat)
    return obj


def create_scene():
    reset_scene()
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)

    gripper_mat = material("gripper_merged_cyan", (0.18, 0.72, 0.84, 1.0))
    bracket_mat = material("manual_align_magenta", (0.85, 0.10, 0.65, 1.0))
    label_mat = material("label_yellow", (1.0, 0.82, 0.10, 1.0))

    gripper_col = collection("gripper_as_one")
    manual_col = collection("manual_align_optional_3mf")

    gripper_files = [
        path
        for path in MESH_DIR.iterdir()
        if path.is_file() and path.suffix.lower() == ".stl" and "V3" in path.name
    ]
    global_min = [float("inf")] * 3
    global_max = [float("-inf")] * 3
    for path in gripper_files:
        mins, maxs = stl_bounds(path)
        for axis in range(3):
            global_min[axis] = min(global_min[axis], mins[axis])
            global_max[axis] = max(global_max[axis], maxs[axis])

    # Origin is the bottom center of the complete gripper assembly. Keep this
    # stable so the object can be appended and manually snapped to ID5 once.
    pivot_mm = (
        (global_min[0] + global_max[0]) / 2.0,
        (global_min[1] + global_max[1]) / 2.0,
        global_min[2],
    )

    vertices = []
    faces = []
    for path in sorted(gripper_files, key=lambda item: item.name):
        part_vertices, part_faces = load_binary_stl_triangles(path, pivot_mm)
        offset = len(vertices)
        vertices.extend(part_vertices)
        faces.extend(tuple(idx + offset for idx in face) for face in part_faces)

    mesh = bpy.data.meshes.new("gripper_visual_merged_mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    gripper = bpy.data.objects.new("gripper_visual_merged", mesh)
    gripper_col.objects.link(gripper)
    assign(gripper, gripper_mat)

    write_binary_stl(MERGED_STL, "gripper_visual_merged", vertices, faces)

    for path in sorted(MESH_DIR.iterdir(), key=lambda item: item.name):
        if path.is_file() and path.suffix.lower() == ".3mf" and not path.name.startswith("arm_link_4_id4_to_id5_visual"):
            create_3mf_manual_object(path, bracket_mat, manual_col)

    note = add_text(
        "gripper_merge_note",
        "Append gripper_as_one. Use fixed ID5 first; 3MF bracket is optional manual align.",
        (0.0, -0.15, 0.14),
        label_mat,
    )
    move_to_collection(note, gripper_col)

    bpy.ops.object.light_add(type="AREA", location=(0.0, -0.45, 0.35))
    light = bpy.context.object
    light.name = "gripper_softbox"
    light.data.energy = 420
    light.data.size = 0.5
    bpy.ops.object.camera_add(location=(0.25, -0.32, 0.20), rotation=(math.radians(62), 0.0, math.radians(38)))
    bpy.context.scene.camera = bpy.context.object

    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_PATH))


if __name__ == "__main__":
    create_scene()
