from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # headless mode is False to visualize the simulation

from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers.pick_place_controller import PickPlaceController
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils import extensions, prims, rotations, viewports
from omni.isaac.nucleus import get_assets_root_path
from omni.physx.scripts import utils
from pxr import Gf, UsdGeom
import numpy as np
import rosgraph
import carb
import sys
import omni
import random

EXTENSIONS = [
    'omni.anim.timeline',
    'omni.anim.people',
    'omni.kit.scripting',
    'omni.anim.curve.core',
    'omni.anim.curve.bundle',
    'omni.anim.curve_editor',
    'omni.isaac.ros_bridge',
    'omni.anim.navigation.bundle',
    'omni.anim.graph.bundle',
]   

for exts in EXTENSIONS:
    extensions.enable_extension(exts)

simulation_app.update()

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

BACKGROUND_STAGE_PATH = "/background"
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

viewports.set_camera_view(eye=np.array([5, 4.2, 4]), target=np.array([-50, 4, -15]))

world = World()
add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

franka = world.scene.add(Franka(prim_path=FRANKA_STAGE_PATH,
                                name="franka",
                                position=np.array([-1.92, 4.12, 1.163])))


camera_prim1 = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim("/World/camera1", "Camera"))
xform_api = UsdGeom.XformCommonAPI(camera_prim1)
#xform_api.SetTranslate(Gf.Vec3d(2, 4.2, 4))
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


i = 0
delay = 0
flag = 0 # 0 for moving from initial to goal position, 1 for moving from goal to initial position
reset_needed = False

while simulation_app.is_running():
    world.step(render=True)
    if world.is_stopped() and not reset_needed:
        reset_needed = True
    if world.is_playing():
        if reset_needed:
            world.reset()
            controller.reset()
            reset_needed = False
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

