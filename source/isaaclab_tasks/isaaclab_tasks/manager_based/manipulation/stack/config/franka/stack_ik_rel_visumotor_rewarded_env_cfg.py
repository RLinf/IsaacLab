# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

##
# Pre-defined configs
##

from isaaclab_assets.robots.cartpole import CARTPOLE_CFG  # isort:skip
from isaaclab_tasks.manager_based.manipulation.stack.config.franka.stack_ik_rel_visuomotor_env_cfg import (
    FrankaCubeStackVisuomotorEnvCfg,
)  # isort:skip
from isaaclab_tasks.manager_based.manipulation.stack import mdp  # isort:skip

from isaaclab.devices.device_base import DeviceBase, DevicesCfg
from isaaclab.devices.keyboard import Se3KeyboardCfg
from isaaclab.devices.gamepad import Se3GamepadCfg
from isaaclab.devices.openxr.openxr_device import OpenXRDeviceCfg
from isaaclab.devices.openxr.retargeters.manipulator.gripper_retargeter import (
    GripperRetargeterCfg,
)
from isaaclab.devices.openxr.retargeters.manipulator.se3_rel_retargeter import (
    Se3RelRetargeterCfg,
)

##
# Scene definition
##


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    success = RewTerm(func=mdp.cubes_stacked, weight=20.0)


##
# Environment configuration
##


@configclass
class FrankaCubeStackVisuomotorRewardedEnvCfg(FrankaCubeStackVisuomotorEnvCfg):
    rewards: RewardsCfg = RewardsCfg()  #add reward terms

    def __post_init__(self):
        super().__post_init__()
        self.sim.render_interval = 5   # set render interval as the same as the decimation interval