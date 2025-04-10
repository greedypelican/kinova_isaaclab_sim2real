# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import argparse

import carb
import numpy as np
import omni.appwindow  # Contains handle to keyboard
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from gen3 import Gen3ReachPolicy
from isaacsim.storage.native import get_assets_root_path


parser = argparse.ArgumentParser(description="Define the number of robots.")
parser.add_argument("--num-robots", type=int, default=1, help="Number of robots (default: 1)")
parser.add_argument(
    "--env-url",
    default="/Isaac/Environments/Grid/default_environment.usd",
    required=False,
    help="Path to the environment url",
)
args = parser.parse_args()
print(f"Number of robots: {args.num_robots}")

first_step = True
reset_needed = False
# initialize robot on first step, run robot advance
def on_physics_step(step_size) -> None:
    global first_step
    global reset_needed
    if first_step:
        robot.initialize()
        first_step = False
    elif reset_needed:
        my_world.reset(True)
        reset_needed = False
        first_step = True
    else:
        robot.forward(step_size, target_command)


# spawn world
my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=8 / 200)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

# spawn warehouse scene
prim = define_prim("/World/Ground", "Xform")
asset_path = assets_root_path + args.env_url
prim.GetReferences().AddReference(asset_path)

# spawn robot
gen3 = Gen3ReachPolicy(
	prim_path="/World/Gen3",
	name="Gen3",
	usd_path=assets_root_path + "/Isaac/Robots/Kinova/Gen3/gen3n7_instanceable.usd",
	position=np.array([0, 0, 0]),
)

robot = gen3

my_world.reset()
my_world.add_physics_callback("physics_step", callback_fn=on_physics_step)

# robot command
target_command = np.zeros(7)

i=0

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped():
        reset_needed = True
    if my_world.is_playing():
    	target_command = np.array([0.5, 0.0, 0.2, 0.7071, 0.0, 0.7071, 0.0])
    	i = i+1
        

simulation_app.close()

