# License: see [LICENSE, LICENSES/legged_gym/LICENSE]

import os
from legged_gym import LEGGED_GYM_ROOT_DIR, LEGGED_GYM_ENVS_DIR
from .base.quadruped import Quadruped
from .base.quadruped_config import LeggedRobotCfg, LeggedRobotCfgPPO
from legged_gym.utils.task_registry import task_registry
task_registry.register( "quadruped", Quadruped, LeggedRobotCfg(), LeggedRobotCfgPPO())

