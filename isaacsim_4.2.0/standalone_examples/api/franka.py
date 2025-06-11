from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # headless mode is False to visualize the simulation

from omni.isaac.core import World
from omni.isaac.franka import Franka
import numpy as np

FRANKA_STAGE_PATH = "/World/Franka"

world = World()
world.scene.add_default_ground_plane()  # Add a ground plane to the scene
franka = world.scene.add(Franka(prim_path=FRANKA_STAGE_PATH,
                                name="franka"))

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