from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # headless mode is False to visualize the simulation

from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils import extensions, prims
from omni.isaac.nucleus import get_assets_root_path
import numpy as np
import rosgraph
import carb
import sys

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
world.scene.add_default_ground_plane()  # Add a ground plane to the scene
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


i = 0
reset_needed = False

while simulation_app.is_running():
    world.step(render=True)  # Step the world, render the scene
    if world.is_stopped() and not reset_needed:
        reset_needed = True
    
    if world.is_playing():
        if reset_needed:
            world.reset()
            reset_needed = False
        i += 1


simulation_app.close()  # Close Isaac Sim

