from typing import Dict, Optional, Sequence, Tuple,Any, Dict, Protocol
from dataclasses import dataclass
import numpy as np
from gello.agents.agent import Agent

from fashionstar_uart_sdk.uart_pocket_handler import (
    PortHandler 
    # SyncPositionControlOptions,
)
from gello.robots.fashionstar import FashionstarRobot


@dataclass
class FireflyRobotConfig:
    joint_ids: Sequence[int]

    gripper_config: Tuple[int, float, float]

    joint_signs: Sequence[int]

    start_joints: Sequence[float]

    def __post_init__(self):
        # self.port_handler = PortHandler

    def make_robot(
        self, port: str = "/dev/ttyUSB0", start_joints: Optional[np.ndarray] = None
    ) -> DynamixelRobot:
        return DynamixelRobot(
            joint_ids=self.joint_ids,
            joint_offsets=list(self.joint_offsets),
            real=True,
            joint_signs=list(self.joint_signs),
            port=port,
            gripper_config=self.gripper_config,
            start_joints=start_joints,
        )


FireflyConfig: Dict[str,FireflyRobotConfig                                                                                                                                                              ] = {
    "piper": ()
}


class Firefly_SigleAgent(Agent):
    def __init__(
            self, 
            port: str,
            firefly_config:FireflyRobotConfig
    ):
        pass

    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        monitor_data = self.port_handler.sync_read["Monitor"](servos_id)
        return np.zeros(self.num_dofs)


# class Firefly_BimanualAgent(Agent):
#     def __init__(self, agent_left: Agent, agent_right: Agent):
#         self.agent_left = agent_left
#         self.agent_right = agent_right

#     def act(self, obs: Dict[str, Any]) -> np.ndarray:
#         left_obs = {}
#         right_obs = {}
#         for key, val in obs.items():
#             L = val.shape[0]
#             half_dim = L // 2
#             assert L == half_dim * 2, f"{key} must be even, something is wrong"
#             left_obs[key] = val[:half_dim]
#             right_obs[key] = val[half_dim:]
#         return np.concatenate(
#             [self.agent_left.act(left_obs), self.agent_right.act(right_obs)]
#         )
