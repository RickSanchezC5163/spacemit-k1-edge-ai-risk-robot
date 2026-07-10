import math
from pathlib import Path

import bpy


PKG_DIR = Path(__file__).resolve().parents[1]
OUT_PATH = PKG_DIR / "blender" / "tracked_robot_preview.blend"


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


def cyl(name, loc, radius, depth, mat, axis="Z", vertices=48):
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


def add_track_treads(side_y, mat):
    for i in range(14):
        x = -0.125 + i * (0.25 / 13.0)
        cube(f"track_tread_top_{side_y}_{i}", (x, side_y, 0.084), (0.010, 0.045, 0.007), mat)
        cube(f"track_tread_bottom_{side_y}_{i}", (x, side_y, 0.000), (0.010, 0.045, 0.007), mat)


def create_model():
    reset_scene()

    mat_track = material("track_black", (0.01, 0.01, 0.012, 1.0))
    mat_chassis = material("purple_chassis", (0.45, 0.30, 0.82, 1.0))
    mat_metal = material("brushed_metal", (0.72, 0.72, 0.68, 1.0))
    mat_arm = material("arm_black", (0.03, 0.03, 0.035, 1.0))
    mat_servo = material("servo_purple", (0.42, 0.29, 0.86, 1.0))
    mat_sensor = material("sensor_black", (0.02, 0.02, 0.022, 1.0))
    mat_gripper = material("gripper_cyan", (0.28, 0.78, 0.86, 1.0))
    mat_board = material("electronics_green", (0.10, 0.45, 0.22, 1.0))
    mat_lens = material("lens_dark", (0.0, 0.0, 0.0, 1.0))

    chassis_length = 0.27
    chassis_width = 0.27
    chassis_top_z = 0.10
    chassis_height = 0.03
    chassis_center_z = chassis_top_z - chassis_height / 2.0

    track_length = 0.27
    track_width = 0.04
    track_height = 0.08
    track_y = 0.23 / 2.0

    # Tracks and chassis.
    cube("left_track_collision_shape", (0.0, track_y, track_height / 2.0), (track_length, track_width, track_height), mat_track)
    cube("right_track_collision_shape", (0.0, -track_y, track_height / 2.0), (track_length, track_width, track_height), mat_track)
    add_track_treads(track_y, mat_track)
    add_track_treads(-track_y, mat_track)

    cube("metal_base_plate", (0.0, 0.0, chassis_center_z), (chassis_length, chassis_width, chassis_height), mat_metal)
    cube("purple_upper_deck", (-0.035, 0.0, 0.118), (0.18, 0.18, 0.018), mat_chassis)
    cube("electronics_stack_placeholder", (0.035, -0.035, 0.145), (0.080, 0.080, 0.060), mat_board)

    # Visible track wheels.
    for side_name, side_y in [("left", track_y), ("right", -track_y)]:
        for x in [-0.105, 0.105]:
            cyl(f"{side_name}_large_idler_{x}", (x, side_y, 0.065), 0.020, 0.040, mat_metal, axis="Y")
        for idx, x in enumerate([-0.09, -0.03, 0.03, 0.09], start=1):
            cyl(f"{side_name}_road_wheel_{idx}", (x, side_y, 0.020), 0.020, 0.035, mat_metal, axis="Y")
            cube(f"{side_name}_suspension_strut_{idx}", (x, side_y, 0.055), (0.012, 0.010, 0.035), mat_metal)

    # Front motors visible from the vehicle photo.
    cyl("left_front_motor_placeholder", (0.110, 0.050, 0.070), 0.018, 0.055, mat_sensor, axis="Y")
    cyl("right_front_motor_placeholder", (0.110, -0.050, 0.070), 0.018, 0.055, mat_sensor, axis="Y")

    # Sensors.
    cyl("n10p_lidar", (0.075, 0.0, 0.130), 0.035, 0.045, mat_sensor, axis="Z")
    cube("d435_body", (0.105, 0.0, 0.110), (0.025, 0.095, 0.028), mat_sensor)
    for y in [-0.030, 0.0, 0.030]:
        cyl(f"d435_lens_{y}", (0.119, y, 0.110), 0.006, 0.004, mat_lens, axis="X", vertices=24)

    # Arm, current simplified serial kinematics with dual-tube main boom.
    arm_base_x = chassis_length / 2.0 - 0.14
    arm_base_z = 0.13
    id1_to_id2 = 0.19
    id2_to_id3 = 0.04
    id3_to_id4 = 0.19

    cyl("arm_id1_round_base", (arm_base_x, 0.0, arm_base_z - 0.018), 0.050, 0.036, mat_arm, axis="Z")
    cube("arm_id1_servo_box", (arm_base_x, 0.0, arm_base_z + 0.020), (0.070, 0.070, 0.030), mat_servo)
    cube("arm_vertical_column_id1_to_id2", (arm_base_x, 0.0, arm_base_z + id1_to_id2 / 2.0), (0.040, 0.040, id1_to_id2), mat_arm)
    id2 = (arm_base_x, 0.0, arm_base_z + id1_to_id2)
    cube("arm_short_link_id2_to_id3", (id2[0] + id2_to_id3 / 2.0, 0.0, id2[2]), (id2_to_id3, 0.050, 0.055), mat_arm)
    id3_x = id2[0] + id2_to_id3
    cyl("arm_upper_tube_id3_to_id4", (id3_x + id3_to_id4 / 2.0, 0.0, id2[2] + 0.026), 0.014, id3_to_id4, mat_arm, axis="X")
    cyl("arm_lower_tube_id3_to_id4", (id3_x + id3_to_id4 / 2.0, 0.0, id2[2] - 0.026), 0.014, id3_to_id4, mat_arm, axis="X")
    id4_x = id3_x + id3_to_id4
    cube("arm_id4_wrist_servo", (id4_x, 0.0, id2[2]), (0.050, 0.055, 0.045), mat_servo)
    cube("gripper_palm", (id4_x + 0.055, 0.0, id2[2]), (0.055, 0.070, 0.030), mat_gripper)
    cube("gripper_left_finger", (id4_x + 0.120, 0.036, id2[2]), (0.080, 0.014, 0.024), mat_gripper)
    cube("gripper_right_finger", (id4_x + 0.120, -0.036, id2[2]), (0.080, 0.014, 0.024), mat_gripper)

    # Reference ground and axes.
    cube("ground_reference", (0.0, 0.0, -0.002), (0.55, 0.45, 0.004), material("ground_gray", (0.35, 0.35, 0.35, 0.22)))
    cyl("x_axis_front", (0.22, 0.0, 0.005), 0.003, 0.18, material("axis_red", (0.9, 0.1, 0.1, 1.0)), axis="X", vertices=16)
    cyl("y_axis_left", (0.0, 0.20, 0.005), 0.003, 0.16, material("axis_green", (0.1, 0.7, 0.1, 1.0)), axis="Y", vertices=16)

    # Camera and lights.
    bpy.ops.object.light_add(type="AREA", location=(0.0, -0.7, 0.8))
    light = bpy.context.object
    light.name = "large_softbox"
    light.data.energy = 550
    light.data.size = 0.7

    bpy.ops.object.camera_add(location=(0.55, -0.55, 0.35), rotation=(math.radians(60), 0.0, math.radians(42)))
    bpy.context.scene.camera = bpy.context.object

    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_PATH))


if __name__ == "__main__":
    create_model()
