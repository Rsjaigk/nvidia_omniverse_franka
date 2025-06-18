from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # headless mode is False to visualize the simulation

from isaacsim.core.utils import extensions, prims, rotations, viewports

EXTENSIONS = [
    'omni.anim.timeline',
    'omni.kit.scripting',
    'isaacsim.ros1.bridge',
]

for exts in EXTENSIONS:
    extensions.enable_extension(exts)

simulation_app.update()

from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
from isaacsim.core.utils.transformations import pose_from_tf_matrix
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.storage.native import get_assets_root_path
from omni.physx.scripts import utils
from pxr import Gf, UsdGeom, Usd
import omni.graph.core as og
import usdrt.Sdf
import numpy as np
import rosgraph
import carb
import sys
import omni
import random
import rospy
from geometry_msgs.msg import Pose
from typing import Tuple

# Initialize the ROS node
rospy.init_node("hand_pose_publisher", anonymous=True)

# Create a publisher for each hand
hand_publishers = [
    rospy.Publisher(f"/hand{idx+1}/pose", Pose, queue_size=10)
    for idx in range(3)
]

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

BACKGROUND_STAGE_PATH = "/World/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

FRANKA_STAGE_PATH = "/World/Franka"

TABLE_STAGE_PATH = "/World/table"
TABLE_USD_PATH = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Furniture/DiningSets/EastRural/EastRural_Table.usd"
BIN_USD_PATH = assets_root_path + "/Isaac/Props/KLT_Bin/small_KLT.usd"

ORANGE_USD_PATH = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Decor/Tchotchkes/Orange_02.usd"
POMEGRENATE_USD_PATH = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Food/Fruit/Pomegranate01.usd"

WORKER1_USD_PATH = assets_root_path + "/Isaac/People/Characters/male_adult_construction_05_new/male_adult_construction_05_new.usd"
WORKER2_USD_PATH = assets_root_path + "/Isaac/People/Characters/male_adult_construction_01_new/male_adult_construction_01_new.usd"
WORKER3_USD_PATH = assets_root_path + "/Isaac/People/Characters/male_adult_construction_05_new/male_adult_construction_05_new.usd"
WORKER4_USD_PATH = assets_root_path + "/Isaac/People/Characters/original_male_adult_construction_01/male_adult_construction_01.usd"
DOCTOR_USD_PATH = assets_root_path + "/Isaac/People/Characters/original_male_adult_medical_01/male_adult_medical_01.usd"
POLICE_USD_PATH = assets_root_path + "/Isaac/People/Characters/original_female_adult_police_02/female_adult_police_02.usd"
BIPED_SETUP_USD_PATH = assets_root_path + "/Isaac/People/Characters/Biped_Setup.usd"
HAND_PATH = "omniverse://localhost/custom_assets/humanhand.usd"
CUBE_USD_PATH = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac/Props/Blocks/nvidia_cube.usd"

viewports.set_camera_view(eye=np.array([5, 4.2, 4]), target=np.array([-50, 4, -15]))

world = World()
add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

franka = world.scene.add(Franka(prim_path=FRANKA_STAGE_PATH,
                                name="franka",
                                position=np.array([-1.92, 4.12, 1.163])))


try:
    og.Controller.edit(
        {"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("IsaacCreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("ROS1CameraHelper", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishJointState", "isaacsim.ros1.bridge.ROS1PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros1.bridge.ROS1SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishTF", "isaacsim.ros1.bridge.ROS1PublishTransformTree"),
                ("PublishClock", "isaacsim.ros1.bridge.ROS1PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "IsaacCreateRenderProduct.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                ("IsaacCreateRenderProduct.outputs:execOut", "ROS1CameraHelper.inputs:execIn"),
                ("IsaacCreateRenderProduct.outputs:renderProductPath", "ROS1CameraHelper.inputs:renderProductPath"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Setting the /Franka target prim to Articulation Controller node
                ("ArticulationController.inputs:robotPath", FRANKA_STAGE_PATH),
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(FRANKA_STAGE_PATH)]),
                ("PublishTF.inputs:targetPrims", [
                    usdrt.Sdf.Path(FRANKA_STAGE_PATH),
                    usdrt.Sdf.Path("/World/Hand/Hand1"),
                    usdrt.Sdf.Path("/World/Hand/Hand2"),
                    usdrt.Sdf.Path("/World/Hand/Hand3")
                ]), 
                ("IsaacCreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path("/World/camera1")]),
            ],
        },
    )
except Exception as e:
    print(e)

simulation_app.update()

camera_prim1 = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim("/World/camera1", "Camera"))
xform_api = UsdGeom.XformCommonAPI(camera_prim1)
xform_api.SetTranslate(Gf.Vec3d(0, 4.1, 3))
xform_api.SetRotate((26, -0.0, 90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
camera_prim1.GetHorizontalApertureAttr().Set(36)     # mm
camera_prim1.GetVerticalApertureAttr().Set(20.25)    # mm (for 16:9 aspect ratio)
camera_prim1.GetProjectionAttr().Set("perspective")

camera_prim1.GetFocalLengthAttr().Set(35)            # mm — sharper, less distortion than 24mm
camera_prim1.GetFocusDistanceAttr().Set(2500)        # mm (2.5 meters — adjust to your scene)


camera_prim2 = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim("/World/camera2", "Camera"))
xform_api = UsdGeom.XformCommonAPI(camera_prim2)
xform_api.SetTranslate(Gf.Vec3d(-1.5, 7.7, 2.75))
xform_api.SetRotate((72, -0.0, 180), UsdGeom.XformCommonAPI.RotationOrderXYZ)
camera_prim2.GetHorizontalApertureAttr().Set(21)
camera_prim2.GetVerticalApertureAttr().Set(16)
camera_prim2.GetProjectionAttr().Set("perspective")
camera_prim2.GetFocalLengthAttr().Set(24)
camera_prim2.GetFocusDistanceAttr().Set(400)

camera_prim3 = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim("/World/camera3", "Camera"))
xform_api = UsdGeom.XformCommonAPI(camera_prim3)
xform_api.SetTranslate(Gf.Vec3d(9.4, -11.5, 6.3))
xform_api.SetRotate((68.5, -0.0, 36), UsdGeom.XformCommonAPI.RotationOrderXYZ)
camera_prim3.GetHorizontalApertureAttr().Set(21)
camera_prim3.GetVerticalApertureAttr().Set(16)
camera_prim3.GetProjectionAttr().Set("perspective")
camera_prim3.GetFocalLengthAttr().Set(24)
camera_prim3.GetFocusDistanceAttr().Set(400)

table = prims.create_prim(
    TABLE_STAGE_PATH,
    "Xform",
    position=np.array([-1.461, 4.113, 0.00]),
    scale=np.array([0.014, 0.013, 0.01522]),
    usd_path=TABLE_USD_PATH,
)

table_prim = world.stage.GetPrimAtPath(TABLE_STAGE_PATH)
franka_prim = world.stage.GetPrimAtPath("/World/Franka")

utils.setRigidBody(table_prim, "convexDecomposition", False)


left_bin = prims.create_prim(
    "/World/Bins/binleft",
    "Xform",
    position=np.array([-1.64, 3.6, 1.22]),
    scale=np.array([2, 2, 0.8]),
    usd_path=BIN_USD_PATH,
)

middle_bin = prims.create_prim(
    "/World/Bins/binmiddle",
    "Xform",
    position=np.array([-1.180, 4.13, 1.22]),
    scale=np.array([2, 2, 0.8]),
    usd_path=BIN_USD_PATH,
)

right_bin = prims.create_prim(
    "/World/Bins/binright",
    "Xform",
    position=np.array([-1.65, 4.6, 1.22]),
    scale=np.array([2, 2, 0.8]),
    usd_path=BIN_USD_PATH,
)


left_bin_human = prims.create_prim(
    "/World/Bins/binlefthuman",
    "Xform",
    position=np.array([-1.12, 3.52, 1.22]),
    scale=np.array([1.5, 1.5, 0.8]),
    usd_path=BIN_USD_PATH,
)

right_bin_human = prims.create_prim(
    "/World/Bins/binrighthuman",
    "Xform",
    position=np.array([-1.12, 4.73, 1.22]),
    scale=np.array([1.5, 1.5, 0.8]),
    usd_path=BIN_USD_PATH,
)

orange1 = prims.create_prim(
    "/World/Fruits/orange1",
    "Xform",
    position=np.array([-1.722, 3.69, 1.19]),
    scale=np.array([0.008, 0.008, 0.008]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
    usd_path=ORANGE_USD_PATH,
)

orange2 = prims.create_prim(
    "/World/Fruits/orange2",
    "Xform",
    position=np.array([-1.722, 3.506, 1.19]),
    scale=np.array([0.008, 0.008, 0.008]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
    usd_path=ORANGE_USD_PATH,
)

orange3 = prims.create_prim(
    "/World/Fruits/orange3",
    "Xform",
    position=np.array([-1.60, 3.60, 1.19]),
    scale=np.array([0.008, 0.008, 0.008]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
    usd_path=ORANGE_USD_PATH,
)

pomegranate1 = prims.create_prim(
    "/World/Fruits/pomegranate1",
    "Xform",
    position=np.array([-1.70, 4.514, 1.18]),
    scale=np.array([0.005, 0.005, 0.005]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 0), 90)),
    usd_path=POMEGRENATE_USD_PATH,
)

pomegranate2 = prims.create_prim(
    "/World/Fruits/pomegranate2",
    "Xform",
    position=np.array([-1.70, 4.684, 1.18]),
    scale=np.array([0.005, 0.005, 0.005]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 60)),
    usd_path=POMEGRENATE_USD_PATH,
)

pomegranate3 = prims.create_prim(
    "/World/Fruits/pomegranate3",
    "Xform",
    position=np.array([-1.58, 4.603, 1.18]),
    scale=np.array([0.005, 0.005, 0.005]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 0), 90)),
    usd_path=POMEGRENATE_USD_PATH,
)


cube_1 = prims.create_prim(
        "/World/Cube/cube1",
        "Xform",
        position=np.array([-1.12, 3.41, 1.19]),
        scale=np.array([0.8, 0.8, 0.8]),
        usd_path=CUBE_USD_PATH,
    )

cube_2 = prims.create_prim(
        "/World/Cube/cube2",
        "Xform",
        position=np.array([-1.12, 3.51, 1.19]),
        scale=np.array([0.8, 0.8, 0.8]),
        usd_path=CUBE_USD_PATH,
    )
cube_3 = prims.create_prim(
        "/World/Cube/cube3",
        "Xform",
        position=np.array([-1.12, 3.61, 1.19]),
        scale=np.array([0.8, 0.8, 0.8]),
        usd_path = CUBE_USD_PATH,
    )

hands1 = prims.create_prim(
    "/World/Hand/Hand1",
    "Xform",
    position=(-0.5, 0.7, 1.5),
    scale=(0.0003, 0.0003, 0.0003),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
    usd_path=HAND_PATH,
)


hands2 = prims.create_prim(
    "/World/Hand/Hand2",
    "Xform",
    position=(1, 4.5, 1.2),
    scale=(0.0003, 0.0003, 0.0003),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
    usd_path=HAND_PATH,
)

# Create the hand prim
hands3 = prims.create_prim(
    "/World/Hand/Hand3",
    "Xform",
    position=(-0.3, 7.3, 1.8),
    scale=(0.0003, 0.0003, 0.0003),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
    usd_path=HAND_PATH,
)

character1 = prims.create_prim(
    "/World/Characters/character1",
    "Xform",
    position=np.array([-1.461, -5.183, 0.00]),
    scale=np.array([1, 1, 1]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 180)),
    usd_path=WORKER1_USD_PATH,
)

character2 = prims.create_prim(
    "/World/Characters/character2",
    "Xform",
    position=np.array([-6.58, 4.113, 0.00]),
    scale=np.array([1, 1, 1]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
    usd_path=WORKER2_USD_PATH,
)

character3 = prims.create_prim(
    "/World/Characters/character3",
    "Xform",
    position=np.array([-1.461, 15.659, 0.00]),
    scale=np.array([1, 1, 1]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 0), 90)),
    usd_path=WORKER3_USD_PATH,
)

character4 = prims.create_prim(
    "/World/Characters/character4",
    "Xform",
    position=np.array([4.298, 4.113, 0.00]),
    scale=np.array([1, 1, 1]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), -90)),
    usd_path=WORKER4_USD_PATH,
)

character5 = prims.create_prim(
    "/World/Characters/character5",
    "Xform",
    position=np.array([-4.34, 12.678, 0.00]),
    scale=np.array([1, 1, 1]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 0), 90)),
    usd_path=DOCTOR_USD_PATH,
)

character6 = prims.create_prim(
    "/World/Characters/character6",
    "Xform",
    position=np.array([8.186, -5.37, 0.00]),
    scale=np.array([1, 1, 1]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 180)),
    usd_path=POLICE_USD_PATH,
)

character7 = prims.create_prim(
    "/World/Characters/Biped_Setup",
    "Xform",
    position=np.array([0, 0, 0]),
    scale=np.array([1, 1, 1]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 180)),
    usd_path=BIPED_SETUP_USD_PATH,
)


world.stage.GetPrimAtPath("/World/Characters/Biped_Setup").GetAttribute("visibility").Set("invisible")

utils.setRigidBody(world.stage.GetPrimAtPath("/World/Fruits/orange1"), "boundingSphere", False)
utils.setRigidBody(world.stage.GetPrimAtPath("/World/Fruits/orange2"), "boundingSphere", False)
utils.setRigidBody(world.stage.GetPrimAtPath("/World/Fruits/orange3"), "boundingSphere", False)
utils.setRigidBody(world.stage.GetPrimAtPath("/World/Fruits/pomegranate1"), "boundingSphere", False)
utils.setRigidBody(world.stage.GetPrimAtPath("/World/Fruits/pomegranate2"), "boundingSphere", False)
utils.setRigidBody(world.stage.GetPrimAtPath("/World/Fruits/pomegranate3"), "boundingSphere", False)

world.stage.GetPrimAtPath("/World/Fruits/orange1").GetAttribute("physxRigidBody:contactSlopCoefficient").Set(0.01)
world.stage.GetPrimAtPath("/World/Fruits/orange2").GetAttribute("physxRigidBody:contactSlopCoefficient").Set(0.01)
world.stage.GetPrimAtPath("/World/Fruits/orange3").GetAttribute("physxRigidBody:contactSlopCoefficient").Set(0.01)
world.stage.GetPrimAtPath("/World/Fruits/pomegranate1").GetAttribute("physxRigidBody:contactSlopCoefficient").Set(0.1)
world.stage.GetPrimAtPath("/World/Fruits/pomegranate2").GetAttribute("physxRigidBody:contactSlopCoefficient").Set(0.1)
world.stage.GetPrimAtPath("/World/Fruits/pomegranate3").GetAttribute("physxRigidBody:contactSlopCoefficient").Set(0.1)

def get_prim_position(stage, path):
    prim = stage.GetPrimAtPath(path)
    if prim and prim.HasAttribute("xformOp:translate"):
        position = prim.GetAttribute("xformOp:translate").Get()
        return np.array(position)
    else:
        return None
    
objects = [orange1, orange2, orange3, pomegranate1, pomegranate2, pomegranate3]
prim_paths = ["/World/Fruits/orange1", "/World/Fruits/orange2", "/World/Fruits/orange3", "/World/Fruits/pomegranate1", "/World/Fruits/pomegranate2", "/World/Fruits/pomegranate3"]

initial_pos = {obj: get_prim_position(world.stage, path) for obj, path in zip(objects, prim_paths)}
print(initial_pos)
for obj in objects:
    if obj in [pomegranate1, pomegranate2, pomegranate3]:
        initial_pos[obj][2] += 0.01

world.reset()  # Reset the world to apply changes


franka.initialize()
controller = PickPlaceController(name = "pick_place_controller", gripper = franka.gripper, robot_articulation = franka)

franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)
#print(franka.dof_names)
franka.disable_gravity()
franka.set_stabilization_threshold(0.05)

franka_home = franka.get_joint_positions()
initial_velocities = franka.get_joint_velocities()
#print(franka_home)

goal_positions = [np.array([-1.187, 4.15, 1.1897]), np.array([-1.187, 4.03, 1.1897]), np.array([-1.187, 4.27, 1.1897])]

selected_objects = random.sample(objects, 3)  
task = list(zip(selected_objects, goal_positions))

stage = omni.usd.get_context().get_stage()

prim_paths = ["/World/Cube/cube1", "/World/Cube/cube2", "/World/Cube/cube3"]

initial_pos_cube = {f"cube{i+1}": get_prim_position(stage, path) for i, path in enumerate(prim_paths[:3])}

final_pos_cube = {
    "cube1": np.array([-1.12, 4.63, 1.19]),
    "cube2": np.array([-1.12, 4.73, 1.19]),
    "cube3": np.array([-1.12, 4.83, 1.19]),
}

hand_paths = ["/World/Hand/Hand1", "/World/Hand/Hand2", "/World/Hand/Hand3"]
hand_positions = [get_prim_position(stage, path) for path in hand_paths]   


start_x = end_x = -1.12
start_z = 1.19
hover_z = 1.4
grip_z = 1.27

bin1_ys = [3.41, 3.51, 3.61]
bin2_ys = [4.63, 4.73, 4.83]

speed_factor = 5
cycle_gap = 120 * speed_factor
cycle_duration = 1.5 * cycle_gap
frame_offset = 0
direction_flag = 0
current_frame = 0
action_queue = []

def get_world_pose(prim: Usd.Prim, current_frame: int) -> Tuple[np.ndarray, np.ndarray]:
    """Get the world-space position and orientation (quaternion) of a prim at the current simulation time."""
    transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode(current_frame))
    transform = np.transpose(transform)  # Convert from row-major to column-major
    translation, orientation = pose_from_tf_matrix(transform)
    return translation, orientation

def attach_cube_to_hand(cube_name, hand_path):
    omni.kit.commands.execute(
        "MovePrim",
        path_from=f"/World/Cube/{cube_name}",
        path_to=f"{hand_path}/{cube_name}"
    )

    cube_path = f"{hand_path}/{cube_name}"
    cube = stage.GetPrimAtPath(cube_path)
    cube_xform = UsdGeom.Xformable(cube)

    if not cube_xform:
        print(f"Could not find Xformable for {cube_path}")
        return

    translate_ops = cube_xform.GetOrderedXformOps()
    if translate_ops:
        translate_ops[0].Set(Gf.Vec3f(0.0, 0.0, 0.0))
    else:
        cube_xform.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))

def detach_cube_from_hand(cube_name, hand_path):
    index = int(cube_name[-1]) - 1  # Extract cube index from name (e.g., "cube1" → 0)

    # Choose the correct bin position based on the direction
    bin_y = bin2_ys[index] if direction_flag == 0 else bin1_ys[index]
    drop_position = Gf.Vec3f(-1.12, bin_y, 1.27)

    cube_full_path = f"{hand_path}/{cube_name}"
    new_path = f"/World/Cube/{cube_name}"

    # Move the cube back to /World/Cube
    omni.kit.commands.execute(
        "MovePrim",
        path_from=cube_full_path,
        path_to=new_path,
    )

    # Set its position to the bin drop position
    moved_cube = stage.GetPrimAtPath(new_path)
    cube_xform = UsdGeom.Xformable(moved_cube)
    if cube_xform:
        translate_ops = cube_xform.GetOrderedXformOps()
        if translate_ops:
            translate_ops[0].Set(drop_position)
        else:
            cube_xform.AddTranslateOp().Set(drop_position)


def handle_actions(frame_now):
    for frame, action, child_name, parent in list(action_queue):
        if frame == frame_now:
            if action == "attach":
                attach_cube_to_hand(child_name, parent)
            elif action == "detach":
                detach_cube_from_hand(child_name, parent)


def animate_hands_cubes(index, direction, offset):
    cube_name = f"cube{index+1}"
    cube_path = f"/World/Cube/{cube_name}"
    cube_position = initial_pos_cube[cube_name] if direction == 0 else final_pos_cube[cube_name]
    #print(f"Animating {cube_name} at {cube_position}")
    hand_path = hand_paths[index]
    hand_home_x, hand_home_y, home_z = hand_positions[index]

    from_ys = bin1_ys if direction == 0 else bin2_ys
    to_ys = bin2_ys if direction == 0 else bin1_ys

    from_y = from_ys[index]
    to_y = to_ys[index]

    #print(f"Animating hand {index} from {from_y} to {to_y}")

    hand = stage.GetPrimAtPath(hand_path)
    hand_xform = UsdGeom.Xformable(hand)
    hand_translate = hand_xform.AddTranslateOp() if not hand_xform.GetOrderedXformOps() else hand_xform.GetOrderedXformOps()[0]

    f0 = offset
    f_above = f0 + 20 * speed_factor
    f_dip = f0 + 22 * speed_factor
    f_attach = f0 + 23 * speed_factor
    f_lift = f0 + 30 * speed_factor
    f_move = f0 + 40 * speed_factor 
    f_drop = f0 + 50 * speed_factor
    f_detach = f0 + 51 * speed_factor 
    f_lift_after_drop = f0 + 60 * speed_factor
    f_return = f0 + 80 * speed_factor

    hand_translate.GetAttr().Set(Gf.Vec3f(hand_home_x, hand_home_y, home_z), time=f0)
    hand_translate.GetAttr().Set(Gf.Vec3f(start_x, from_y, hover_z), time=f_above)
    hand_translate.GetAttr().Set(Gf.Vec3f(start_x, from_y, grip_z), time=f_dip)
    hand_translate.GetAttr().Set(Gf.Vec3f(start_x, from_y, hover_z), time=f_lift)
    hand_translate.GetAttr().Set(Gf.Vec3f(end_x, to_y, hover_z), time=f_move)
    hand_translate.GetAttr().Set(Gf.Vec3f(end_x, to_y, grip_z), time=f_drop)
    hand_translate.GetAttr().Set(Gf.Vec3f(end_x, to_y, grip_z), time=f_detach)
    hand_translate.GetAttr().Set(Gf.Vec3f(end_x, to_y, hover_z), time=f_lift_after_drop)
    hand_translate.GetAttr().Set(Gf.Vec3f(hand_home_x, hand_home_y, hover_z), time=f_return)

    # Schedule cube attach/detach
    action_queue.append((f_attach, "attach", cube_name, hand_path))
    action_queue.append((f_detach, "detach", cube_name, hand_path))

def force_detach_all_cubes():
    for i in range(3):
        cube_name = f"cube{i+1}"
        hand_path = hand_paths[i]
        cube_in_hand_path = f"{hand_path}/{cube_name}"
        world_cube_path = f"/World/Cube/{cube_name}"

        prim = stage.GetPrimAtPath(cube_in_hand_path)
        if prim and prim.IsValid():
            omni.kit.commands.execute(
                "MovePrim",
                path_from=cube_in_hand_path,
                path_to=world_cube_path,
            )

def reset_cubes_to_initial_positions():
    for cube_name, initial_position in initial_pos_cube.items():
        cube_path = f"/World/Cube/{cube_name}"
        cube = stage.GetPrimAtPath(cube_path)
        cube_xform = UsdGeom.Xformable(cube)
        
        if cube_xform:
            translate_ops = cube_xform.GetOrderedXformOps()
            if translate_ops:
                translate_ops[0].Set(Gf.Vec3f(initial_position[0], initial_position[1], initial_position[2]))
            else:
                cube_xform.AddTranslateOp().Set(Gf.Vec3f(initial_position[0], initial_position[1], initial_position[2]))

i = 0
delay = 0
flag = 0 # 0 for moving from initial to goal position, 1 for moving from goal to initial position
reset_needed = False

index = random.choice([3, 6, 9, 12, 15])
print(f"Selected index at start: {index}")

while simulation_app.is_running():
    world.step(render=True)
    
    if world.is_stopped() and not reset_needed:
        reset_needed = True
        current_frame = 0
        frame_offset = 0
        direction_flag = 0
        action_queue.clear()
        force_detach_all_cubes()
        reset_cubes_to_initial_positions()

    if world.is_playing():
        if reset_needed:
            world.reset()
            controller.reset()
            reset_needed = False

        current_frame += 1

        handle_actions(current_frame)
            
        for j in range(3):
           animate_hands_cubes(j, direction_flag, frame_offset + j * (cycle_gap//index))

        if current_frame >= frame_offset + cycle_duration:
            direction_flag = 1 - direction_flag
            frame_offset = current_frame
            index = random.choice([3, 6, 9, 12, 15])
            print(f"Selected index: {index}") 

        for hand_index, hand_path in enumerate(hand_paths):
            hand_prim = stage.GetPrimAtPath(hand_path)
            if hand_prim.IsValid():
                pos, rot = get_world_pose(hand_prim, current_frame)

                # Publish to ROS
                pose_msg = Pose()
                pose_msg.position.x = pos[0]
                pose_msg.position.y = pos[1]
                pose_msg.position.z = pos[2]
                pose_msg.orientation.x = rot[0]
                pose_msg.orientation.y = rot[1]
                pose_msg.orientation.z = rot[2]
                pose_msg.orientation.w = rot[3]

                hand_publishers[hand_index].publish(pose_msg)

                #print(f"Published Hand {hand_index+1} Position: {pos}, Rotation: {rot}")
            else:
                print(f"Invalid prim path for hand {hand_index+1}")

        current_joint_positions = franka.get_joint_positions()
        franka_velocities = franka.get_joint_velocities()
        print(franka_velocities)
        obj, goal_pos = task[i]
        picking_position = get_prim_position(world.stage, obj.GetPath())
        if obj in [pomegranate1, pomegranate2, pomegranate3]:
            picking_position[2] += 0.01
        if flag == 0:
            picking_position = initial_pos[obj]
            placing_position = goal_pos
        else:
            picking_position = goal_pos
            placing_position = initial_pos[obj]
        actions = controller.forward(   
            picking_position=picking_position,
            placing_position=placing_position,
            current_joint_positions=current_joint_positions,
        )
        #print(f"Object: {obj}, Picking Position: {picking_position}, Placing Position: {placing_position}")
        franka.apply_action(actions)

        if controller.is_done():
            controller.reset()
            i += 1
            print(f"Task {i} completed")
            if i == 3:
                i = 0
                flag = 1 - flag
                if flag == 0:
                    franka.apply_action(ArticulationAction(joint_positions=franka_home, joint_velocities=initial_velocities))
                    while delay < 10:
                        world.step(render=True)
                        delay += 1                  
                delay = 0

simulation_app.close() # close Isaac Sim

