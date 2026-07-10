import math
import struct
from pathlib import Path

import bpy


PKG_DIR = Path(__file__).resolve().parents[1]
MESH_DIR = PKG_DIR / "meshes" / "gazebo_roscar"
OUT_PATH = PKG_DIR / "blender" / "tracked_robot_sw_import.blend"

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


def cube(name, loc, dims, mat):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=loc)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = dims
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    assign(obj, mat)
    return obj


def cyl(name, loc, radius, depth, mat, axis="Z", vertices=32):
    rotation = (0.0, 0.0, 0.0)
    if axis == "X":
        rotation = (0.0, math.pi / 2.0, 0.0)
    elif axis == "Y":
        rotation = (math.pi / 2.0, 0.0, 0.0)
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=vertices,
        radius=radius,
        depth=depth,
        location=loc,
        rotation=rotation,
    )
    obj = bpy.context.object
    obj.name = name
    assign(obj, mat)
    return obj


def stl_bounds(path):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"{path.name} is too small to be an STL")
    triangles = struct.unpack("<I", data[80:84])[0]
    expected = 84 + triangles * 50
    if expected > len(data):
        raise ValueError(f"{path.name} is not a binary STL")
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


def collection(name):
    col = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(col)
    return col


def move_to_collection(obj, col):
    for current in list(obj.users_collection):
        current.objects.unlink(obj)
    col.objects.link(obj)


def material_for_name(name, mats):
    lower = name.lower()
    if "chassis" in lower or "底盘" in name or "dizuo" in lower:
        return mats["purple"]
    if "track" in lower:
        return mats["track"]
    if "d435" in lower or "lidar" in lower or "雷达" in name:
        return mats["sensor"]
    if "gripper" in lower or "爪" in name or "夹" in name or "尖角" in name:
        return mats["cyan"]
    if "舵机" in name or "servo" in lower:
        return mats["servo"]
    return mats["metal"]


def link_collection_name(filename):
    stem = filename.rsplit(".", 1)[0]
    if " - " in stem:
        return stem.split(" - ", 1)[0].strip()
    return "vehicle_static_visuals"


def import_stl(path, pivot_mm, mats, imported_root):
    before = set(bpy.context.scene.objects)
    if hasattr(bpy.ops.wm, "stl_import"):
        bpy.ops.wm.stl_import(filepath=str(path))
    else:
        bpy.ops.import_mesh.stl(filepath=str(path))
    after = set(bpy.context.scene.objects)
    new_objects = [obj for obj in after - before if obj.type == "MESH"]

    target_col_name = link_collection_name(path.name)
    target_col = bpy.data.collections.get(target_col_name)
    if target_col is None:
        target_col = bpy.data.collections.new(target_col_name)
        imported_root.children.link(target_col)

    for obj in new_objects:
        obj.name = path.stem
        obj.data.name = f"{path.stem}_mesh"
        mat = material_for_name(path.name, mats)
        assign(obj, mat)

        # SolidWorks STL is unitless here, but the measured chassis box proves
        # the exported numbers are millimeters. Convert to a ROS-friendly frame:
        # +X forward, +Y left, +Z up, chassis centered at the ground plane.
        for vertex in obj.data.vertices:
            sw_x, sw_y, sw_z = vertex.co.x, vertex.co.y, vertex.co.z
            vertex.co.x = -(sw_y - pivot_mm[1]) * SCALE
            vertex.co.y = (sw_x - pivot_mm[0]) * SCALE
            vertex.co.z = (sw_z - pivot_mm[2]) * SCALE
        obj.data.update()
        move_to_collection(obj, target_col)

    return new_objects


def add_empty(name, loc, rotation=(0.0, 0.0, 0.0), display_type="ARROWS"):
    bpy.ops.object.empty_add(type=display_type, location=loc, rotation=rotation)
    obj = bpy.context.object
    obj.name = name
    obj.empty_display_size = 0.035
    return obj


def add_text(name, text, loc, size, mat):
    bpy.ops.object.text_add(location=loc, rotation=(math.radians(70), 0.0, math.radians(0)))
    obj = bpy.context.object
    obj.name = name
    obj.data.body = text
    obj.data.align_x = "CENTER"
    obj.data.size = size
    assign(obj, mat)
    return obj


def add_reference_geometry(ref_col, mats):
    chassis_length = 0.27
    chassis_width = 0.27
    chassis_top_z = 0.10
    chassis_height = 0.03
    track_y = 0.23 / 2.0
    track_height = 0.08
    track_width = 0.04

    refs = [
        cube("ref_chassis_27cm", (0.0, 0.0, chassis_top_z - chassis_height / 2.0), (chassis_length, chassis_width, chassis_height), mats["ref"]),
        cube("ref_left_track", (0.0, track_y, track_height / 2.0), (0.27, track_width, track_height), mats["track_ref"]),
        cube("ref_right_track", (0.0, -track_y, track_height / 2.0), (0.27, track_width, track_height), mats["track_ref"]),
        cyl("ref_n10p_lidar_center", (0.075, 0.0, 0.13), 0.03, 0.025, mats["sensor_ref"], axis="Z"),
        cube("ref_d435_center", (0.105, 0.0, 0.11), (0.025, 0.095, 0.028), mats["sensor_ref"]),
        cyl("ref_x_forward_axis", (0.22, 0.0, 0.004), 0.003, 0.20, mats["red"], axis="X", vertices=16),
        cyl("ref_y_left_axis", (0.0, 0.18, 0.004), 0.003, 0.16, mats["green"], axis="Y", vertices=16),
    ]
    for obj in refs:
        move_to_collection(obj, ref_col)


def add_joint_markers(joint_col, mats):
    arm_base_x = 0.135 - 0.14
    id1_z = 0.13
    id2_z = id1_z + 0.19
    id3_x = arm_base_x + 0.04
    id4_x = id3_x + 0.19
    id5_x = id4_x + 0.055

    joints = [
        ("ID1_yaw_Z_limit_pm180", (arm_base_x, 0.0, id1_z), "Z yaw +/-180 deg"),
        ("ID2_pitch_Y_limit_pm90", (arm_base_x, 0.0, id2_z), "Y pitch +/-90 deg"),
        ("ID3_pitch_Y_limit_pm105", (id3_x, 0.0, id2_z), "Y pitch +/-105 deg"),
        ("ID4_wrist_Y_limit_pm90", (id4_x, 0.0, id2_z), "Y wrist +/-90 deg"),
        ("ID5_gripper_limit_0_37", (id5_x, 0.0, id2_z), "gripper 0..37 deg total"),
    ]

    for name, loc, label in joints:
        empty = add_empty(name, loc)
        move_to_collection(empty, joint_col)
        axis = cyl(f"{name}_axis_marker", loc, 0.004, 0.07, mats["yellow"], axis="Z" if "yaw" in name else "Y", vertices=16)
        move_to_collection(axis, joint_col)
        text = add_text(f"{name}_label", label, (loc[0], loc[1], loc[2] + 0.035), 0.012, mats["yellow"])
        move_to_collection(text, joint_col)


def create_scene():
    reset_scene()
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)

    mats = {
        "purple": material("cad_purple", (0.45, 0.30, 0.82, 1.0)),
        "track": material("cad_track_black", (0.01, 0.01, 0.012, 1.0)),
        "sensor": material("cad_sensor_black", (0.02, 0.02, 0.025, 1.0)),
        "cyan": material("cad_gripper_cyan", (0.22, 0.78, 0.88, 1.0)),
        "servo": material("cad_servo_violet", (0.38, 0.28, 0.78, 1.0)),
        "metal": material("cad_metal_gray", (0.70, 0.70, 0.67, 1.0)),
        "ref": material("reference_chassis_translucent", (0.80, 0.55, 0.20, 0.28)),
        "track_ref": material("reference_track_translucent", (0.05, 0.05, 0.05, 0.24)),
        "sensor_ref": material("reference_sensor_blue", (0.10, 0.38, 0.95, 0.45)),
        "red": material("axis_x_red", (0.90, 0.08, 0.08, 1.0)),
        "green": material("axis_y_green", (0.08, 0.70, 0.08, 1.0)),
        "yellow": material("joint_limit_yellow", (1.0, 0.78, 0.08, 1.0)),
    }
    for key in ["ref", "track_ref", "sensor_ref"]:
        mats[key].use_nodes = False
        mats[key].blend_method = "BLEND"

    imported_root = collection("solidworks_meshes_mm_to_ros_m")
    ref_col = collection("reference_measurements")
    joint_col = collection("joint_axes_and_limits")

    chassis_file = MESH_DIR / "chassis_visual.STL"
    mins, maxs = stl_bounds(chassis_file)
    pivot_mm = (
        (mins[0] + maxs[0]) / 2.0,
        (mins[1] + maxs[1]) / 2.0,
        mins[2],
    )

    imported = []
    skipped = []
    for path in sorted(MESH_DIR.iterdir()):
        if path.suffix.lower() == ".stl":
            imported.extend(import_stl(path, pivot_mm, mats, imported_root))
        elif path.suffix.lower() in {".3mf", ".stp", ".step"}:
            skipped.append(path.name)

    add_reference_geometry(ref_col, mats)
    add_joint_markers(joint_col, mats)

    note = "Imported STL: mm -> m, SW low-Y -> ROS +X. 3MF/STEP not imported here."
    text = add_text("import_note", note, (0.0, -0.20, 0.18), 0.012, mats["yellow"])
    move_to_collection(text, joint_col)
    if skipped:
        skip_text = add_text("skipped_meshes", "Skipped: " + ", ".join(skipped), (0.0, -0.20, 0.15), 0.010, mats["yellow"])
        move_to_collection(skip_text, joint_col)

    bpy.ops.object.light_add(type="AREA", location=(0.0, -0.7, 0.7))
    light = bpy.context.object
    light.name = "large_softbox"
    light.data.energy = 650
    light.data.size = 0.8

    bpy.ops.object.camera_add(location=(0.55, -0.62, 0.38), rotation=(math.radians(61), 0.0, math.radians(41)))
    bpy.context.scene.camera = bpy.context.object

    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_PATH))


if __name__ == "__main__":
    create_scene()
