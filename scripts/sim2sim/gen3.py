# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


from typing import Optional

import numpy as np
import omni
import omni.kit.commands
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.storage.native import get_assets_root_path

class Gen3ReachPolicy(PolicyController):
    """The Gen3 arm running Reach Policy"""

    def __init__(
        self,
        prim_path: str,
        root_path: Optional[str] = None,
        name: str = "gen3",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        Initialize Gen3 robot and import reach policy.

        Args:
            prim_path (str) -- prim path of the robot on the stage
            root_path (Optional[str]): The path to the articulation root of the robot
            name (str) -- name of the quadruped
            usd_path (str) -- robot usd filepath in the directory
            position (np.ndarray) -- position of the robot
            orientation (np.ndarray) -- orientation of the robot

        """
        assets_root_path = get_assets_root_path()
        if usd_path == None:
            usd_path = assets_root_path + "/Isaac/Robots/Kinova/Gen3/gen3n7_instanceable.usd"
        super().__init__(name, prim_path, root_path, usd_path, position, orientation)
        self.load_policy(
            # "/home/louis/Documents/simulation/gen3/logs/rsl_rl/reach_gen3/2025-04-10_00-47-35/model_999.pt", 
            "/home/louis/Documents/simulation/gen3/pretrained_models/reach/policy.pt",
            # "/home/louis/Documents/simulation/gen3/logs/rsl_rl/reach_gen3/2025-04-10_00-47-35/params/env.yaml",
            "/home/louis/Documents/simulation/gen3/pretrained_models/reach/env.yaml",
        )
        self._action_scale = 0.5
        self._previous_action = np.zeros(7)
        self._policy_counter = 0
        print(self.robot.dof_names)
        


    def _compute_observation(self, command):						# 7 joint_pos + 7 joint_vel + 7 pose + 7 last_actions = 28
        """
        Compute the observation vector for the policy.

        Argument:
        command (np.ndarray) -- the target pose command (x, y, z, qx, qy, qz, qw) -> 7

        Returns:
        np.ndarray -- The observation vector.

        """
            
        obs = np.zeros(28)
        # Joint states
        current_joint_pos = self.robot.get_joint_positions()
        print("joint pos : " + str(np.round(current_joint_pos, 4)))
        current_joint_vel = self.robot.get_joint_velocities()
        print(self.default_pos)
        obs[:7] = current_joint_pos - self.default_pos
        obs[7:14] = current_joint_vel
        # Command
        obs[14:21] = command
        # Previous Action
        obs[21:28] = self._previous_action
        return obs

    def forward(self, dt, command):	# this one should work just fine
        """
        Compute the desired articulation action and apply them to the robot articulation.

        Argument:
        dt (float) -- Timestep update in the world.
        command (np.ndarray) -- the target pose command (x, y, z, qx, qy, qz, qw) -> 7

        """
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            self.action = self._compute_action(obs)
            self._previous_action = self.action.copy()
            
            
            
        joint_positions=self.default_pos + (self.action * self._action_scale)

        # s = joint_positions[6]
        # joint_positions[6] = joint_positions[5]
        # joint_positions[5] = s

        # print(f"actions processed   : {np.round(joint_positions, 4)}")
        
        action = ArticulationAction(joint_positions=joint_positions)
        self.robot.apply_action(action)
        
        

        self._policy_counter += 1

    def initialize(self):
        """
        Overloads the default initialize function to use default articulation root properties in the USD
        """
        return super().initialize(set_articulation_props=False)
