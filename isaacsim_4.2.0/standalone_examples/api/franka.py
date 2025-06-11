from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # headless mode is False to visualize the simulation

from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers.pick_place_controller import PickPlaceController
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils import extensions, prims
from omni.isaac.nucleus import get_assets_root_path
import numpy as np
import rosgraph
import carb
import sys
import random

extensions.enable_extension("omni.isaac.ros_bridge")

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
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
FRANKA_STAGE_PATH = "/World/Franka"
BIN_USD_PATH = assets_root_path + "/Isaac/Props/KLT_Bin/small_KLT.usd"


world = World()
add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

franka = world.scene.add(Franka(prim_path=FRANKA_STAGE_PATH,
                                name="franka",
                                position=np.array([0, 0, 0.0])))


left_bin = prims.create_prim(
    "/World/binleft",
    "Xform",
    position=np.array([0.289, -0.524, 0.065]),
    scale=np.array([2, 2, 0.8]),
    usd_path=BIN_USD_PATH,
)

middle_bin = prims.create_prim(
    "/World/binmiddle",
    "Xform",
    position=np.array([0.73, 0.015, 0.065]),
    scale=np.array([2, 2, 0.8]),
    usd_path=BIN_USD_PATH,
)

right_bin = prims.create_prim(
    "/World/binright",
    "Xform",
    position=np.array([0.236, 0.48, 0.065]),
    scale=np.array([2, 2, 0.8]),
    usd_path=BIN_USD_PATH,
)

# Add cubes
cube_1 =  world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube1",
        name="cube1",
        position=np.array([0.187, 0.382, 0.0365]),
        scale=np.array([0.05, 0.05, 0.05]),
        color=np.array([0, 0, 1.0]),
    ))

cube_2 =  world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube2",
        name="cube2",
        position=np.array([0.187, 0.551, 0.0365]),
        scale=np.array([0.05, 0.05, 0.05]),
        color=np.array([0, 0, 1.0]),
    ))

cube_3 =  world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube3",
        name="cube3",
        position=np.array([0.290, 0.474, 0.0365]),
        scale=np.array([0.05, 0.05, 0.05]),
        color=np.array([0, 0, 1.0]),
    ))

# Add spheres
sphere_1 = world.scene.add(
    DynamicSphere(
        prim_path="/World/sphere1",
        name="sphere1",
        position=np.array([0.246, -0.462, 0.0365]),
        scale=np.array([0.03, 0.03, 0.03]),
        color=np.array([0, 0, 0.0]),
    ))

sphere_2 = world.scene.add(
    DynamicSphere(
        prim_path="/World/sphere2",
        name="sphere2",
        position=np.array([0.246, -0.620, 0.0365]),
        scale=np.array([0.03, 0.03, 0.03]),
        color=np.array([0, 0, 0.0]),    
    ))

sphere_3 = world.scene.add(
    DynamicSphere(
        prim_path="/World/sphere3",
        name="sphere3",
        position=np.array([0.345, -0.537, 0.0365]),
        scale=np.array([0.03, 0.03, 0.03]),
        color=np.array([0, 0, 0.0]),    
    ))

world.reset()  # Reset the world to apply changes

controller = PickPlaceController(name = "pick_place_controller", gripper = franka.gripper, robot_articulation = franka)

franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)

franka_home = franka.get_joint_positions()
print(franka_home)
initial_velocities = franka.get_joint_velocities()

objects = [cube_1, cube_2, cube_3, sphere_1, sphere_2, sphere_3]
initial_pos = {obj: obj.get_world_pose()[0] for obj in objects}
print(initial_pos)

goal_positions = [np.array([0.72, 0.000, 0.0365]), np.array([0.72, -0.115, 0.0365]), np.array([0.72, 0.115, 0.0365])]

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
        #print(current_joint_positions)
        obj, goal_pos = task[i]
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
            end_effector_offset=np.array([0, 0.005, 0]),
        )

        franka.apply_action(actions)

        if controller.is_done():
            controller.reset()
            i += 1
            if i == 3:
                i = 0
                flag = 1 - flag
                if flag == 0:
                    franka.apply_action(ArticulationAction(joint_positions=franka_home)) 
                    while delay < 5:
                        world.step(render=True)
                        delay += 1                  
                delay = 0

simulation_app.close() # close Isaac Sim

