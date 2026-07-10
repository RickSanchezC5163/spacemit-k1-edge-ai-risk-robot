import math
import struct
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path

import bpy
from mathutils import Vector


PKG_DIR = Path(__file__).resolve().parents[1]
MESH_DIR = PKG_DIR / "meshes" / "gazebo_roscar"
OUT_PATH = PKG_DIR / "blender" / "tracked_robot_new_parts_import.blend"
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


def collection(name, parent=None):
    col = bpy.data.collections.new(name)
    target = parent if parent else bpy.context.scene.collection
    target.children.link(col)
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


def clean_name(path):
    return path.stem.replace("柔性自适应二指机械爪（舵机版本）带硅胶V3 - ", "")


def group_for_name(name):
    if name.startswith("arm_link_4_id3_to_id4_visual"):
        return "01_upper_arm_candidate"
    if "固定机架" in name or "舵机996" in name:
        return "02_gripper_fixed_frame"
    if "小手指模块-1" in name or "驱动杆左" in name or "连杆-dk-1" in name:
        return "03_left_finger_motion_group"
    if "小手指模块-2" in name or "驱动杆右" in name or "连杆-dk-2" in name:
        return "04_right_finger_motion_group"
    if "垫片" in name or "螺栓" in name or "螺母" in name:
        return "05_fasteners_reference"
    return "06_unsorted_new_parts"


def material_for_group(group_name, mats):
    if group_name.startswith("01"):
        return mats["arm"]
    if group_name.startswith("02"):
        return mats["frame"]
    if group_name.startswith("03"):
        return mats["left"]
    if group_name.startswith("04"):
        return mats["right"]
    if group_name.startswith("05"):
        return mats["fastener"]
    return mats["other"]


def import_stl(path, pivot_mm, group_cols, mats):
    before = set(bpy.context.scene.objects)
    bpy.ops.wm.stl_import(filepath=str(path))
    after = set(bpy.context.scene.objects)
    group_name = group_for_name(path.name)
    mat = material_for_group(group_name, mats)
    imported = []
    for obj in after - before:
        if obj.type != "MESH":
            continue
        obj.name = clean_name(path)
        obj.data.name = f"{obj.name}_mesh"
        for vertex in obj.data.vertices:
            vertex.co = ros_from_sw_mm((vertex.co.x, vertex.co.y, vertex.co.z), pivot_mm)
        obj.data.update()
        assign(obj, mat)
        move_to_collection(obj, group_cols[group_name])
        imported.append(obj)
    return imported


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


def import_3mf(path, col, mats):
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
    obj = bpy.data.objects.new(path.stem, mesh)
    obj.location = (0.0, -0.12, 0.04)
    col.objects.link(obj)
    assign(obj, mats["manual"])
    return obj


def add_empty(name, loc, col, display_type="ARROWS"):
    bpy.ops.object.empty_add(type=display_type, location=loc)
    obj = bpy.context.object
    obj.name = name
    obj.empty_display_size = 0.025
    move_to_collection(obj, col)
    return obj


def add_text(name, text, loc, col, mats):
    bpy.ops.object.text_add(location=loc, rotation=(math.radians(70), 0.0, 0.0))
    obj = bpy.context.object
    obj.name = name
    obj.data.body = text
    obj.data.align_x = "CENTER"
    obj.data.size = 0.008
    assign(obj, mats["label"])
    move_to_collection(obj, col)
    return obj


def create_scene():
    reset_scene()
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)

    mats = {
        "arm": material("upper_arm_candidate_black", (0.03, 0.03, 0.035, 1.0)),
        "frame": material("gripper_fixed_frame_gray", (0.55, 0.55, 0.52, 1.0)),
        "left": material("left_finger_cyan", (0.20, 0.75, 0.86, 1.0)),
        "right": material("right_finger_blue", (0.18, 0.38, 0.90, 1.0)),
        "fastener": material("fastener_metal", (0.80, 0.78, 0.70, 1.0)),
        "other": material("unsorted_orange", (0.95, 0.48, 0.15, 1.0)),
        "manual": material("manual_align_magenta", (0.85, 0.10, 0.65, 1.0)),
        "label": material("label_yellow", (1.0, 0.82, 0.10, 1.0)),
    }

    root_col = collection("new_gripper_parts_scaled_mm_to_m")
    group_names = [
        "01_upper_arm_candidate",
        "02_gripper_fixed_frame",
        "03_left_finger_motion_group",
        "04_right_finger_motion_group",
        "05_fasteners_reference",
        "06_unsorted_new_parts",
    ]
    group_cols = {name: collection(name, root_col) for name in group_names}
    manual_col = collection("07_manual_align_3mf", root_col)
    notes_col = collection("08_import_notes_and_axes", root_col)

    stl_files = [
        path
        for path in MESH_DIR.iterdir()
        if path.is_file()
        and path.suffix.lower() == ".stl"
        and ("V3" in path.name or path.name.startswith("arm_link_4_id3_to_id4_visual"))
    ]
    global_min = [float("inf")] * 3
    global_max = [float("-inf")] * 3
    for path in stl_files:
        mins, maxs = stl_bounds(path)
        for axis in range(3):
            global_min[axis] = min(global_min[axis], mins[axis])
            global_max[axis] = max(global_max[axis], maxs[axis])
    pivot_mm = (
        (global_min[0] + global_max[0]) / 2.0,
        (global_min[1] + global_max[1]) / 2.0,
        global_min[2],
    )

    for path in sorted(stl_files, key=lambda item: item.name):
        import_stl(path, pivot_mm, group_cols, mats)

    for path in sorted(MESH_DIR.iterdir(), key=lambda item: item.name):
        if path.is_file() and path.suffix.lower() == ".3mf" and not path.name.startswith("arm_link_4_id4_to_id5_visual"):
            import_3mf(path, manual_col, mats)

    add_empty("gripper_mount_origin_move_this_to_ID5", (0.0, 0.0, 0.0), notes_col)
    add_empty("left_finger_pivot_placeholder", (-0.028, -0.050, 0.078), notes_col)
    add_empty("right_finger_pivot_placeholder", (-0.028, 0.050, 0.078), notes_col)
    add_text(
        "import_rule_note",
        "STL groups keep assembly coordinates. 3MF is local and needs manual alignment.",
        (0.0, -0.16, 0.13),
        notes_col,
        mats,
    )

    bpy.ops.object.light_add(type="AREA", location=(0.0, -0.45, 0.35))
    light = bpy.context.object
    light.name = "new_parts_softbox"
    light.data.energy = 450
    light.data.size = 0.5
    bpy.ops.object.camera_add(location=(0.30, -0.36, 0.22), rotation=(math.radians(62), 0.0, math.radians(38)))
    bpy.context.scene.camera = bpy.context.object

    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_PATH))


if __name__ == "__main__":
    create_scene()
