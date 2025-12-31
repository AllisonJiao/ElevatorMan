# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import os

from isaacsim.core.utils.extensions import get_extension_path_from_name

from source.controllers.rmp_flow import RmpFlowControllerCfg
from source.utils.assets import ISAACLAB_NUCLEUS_DIR

ISAACLAB_NUCLEUS_RMPFLOW_DIR = os.path.join(ISAACLAB_NUCLEUS_DIR, "Controllers", "RmpFlowAssets")

# Note: RMP-Flow config files for supported robots are stored in the motion_generation extension
_RMP_CONFIG_DIR = os.path.join(
    get_extension_path_from_name("isaacsim.robot_motion.motion_generation"), "motion_policy_configs"
)

# Path to current directory
_CUR_DIR = os.path.dirname(os.path.realpath(__file__))

AGIBOT_LEFT_ARM_RMPFLOW_CFG = RmpFlowControllerCfg(
    config_file=os.path.join(ISAACLAB_NUCLEUS_RMPFLOW_DIR, "agibot", "rmpflow", "agibot_left_arm_rmpflow_config.yaml"),
    urdf_file=os.path.join(ISAACLAB_NUCLEUS_RMPFLOW_DIR, "agibot", "agibot.urdf"),
    collision_file=os.path.join(ISAACLAB_NUCLEUS_RMPFLOW_DIR, "agibot", "rmpflow", "agibot_left_arm_gripper.yaml"),
    frame_name="gripper_center",
    evaluations_per_frame=5,
    ignore_robot_state_updates=True,
)

AGIBOT_RIGHT_ARM_RMPFLOW_CFG = RmpFlowControllerCfg(
    config_file=os.path.join(ISAACLAB_NUCLEUS_RMPFLOW_DIR, "agibot", "rmpflow", "agibot_right_arm_rmpflow_config.yaml"),
    urdf_file=os.path.join(ISAACLAB_NUCLEUS_RMPFLOW_DIR, "agibot", "agibot.urdf"),
    collision_file=os.path.join(ISAACLAB_NUCLEUS_RMPFLOW_DIR, "agibot", "rmpflow", "agibot_right_arm_gripper.yaml"),
    frame_name="right_gripper_center",
    evaluations_per_frame=5,
    ignore_robot_state_updates=True,
)

"""Configuration of RMPFlow for Agibot humanoid."""
