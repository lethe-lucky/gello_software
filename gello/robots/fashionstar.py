from typing import Dict, Optional, Sequence, Tuple
import numpy as np

# from gello.robots.robot import Robot
from fashionstar_uart_sdk.uart_pocket_handler import (
    PortHandler 
)

class FashionstarRobot():
    def __init__(self,joint_ids:Sequence[int],gripper_config:Tuple[int, float, float],port:str,baudrate:int):
        self._gripper_id,self._gripper_range_min,self._gripper_range_max = gripper_config
        
        self._portHandler = PortHandler(port,baudrate)
        self._id_to_name = {}

        if gripper_config == (0,0.0,0.0):
            self._servo_ids = tuple(joint_ids)
            for index ,servo_id in enumerate(self._servo_ids):
                self._id_to_name[f'joint_{index}'] = servo_id
        else:
            self._servo_ids = tuple(joint_ids) + ( self._gripper_id,)
            for index ,servo_id in enumerate(self._servo_ids):
                if index == len(self._servo_ids)-1:
                    self._id_to_name[f'gripper'] = servo_id
                else:
                    self._id_to_name[f'joint_{index}'] = servo_id




    
    
    def open(self):
        self._portHandler.openPort()

    def num_dofs(self) -> int:
        return  len(self._servo_ids)

    def get_joint_state(self) -> np.ndarray:
        monitor_data = self._portHandler.sync_read["Monitor"](self._servo_ids)

        return None
 

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        raise NotImplementedError

    def get_observations(self) -> Dict[str, np.ndarray]:
        return {"joint_state": self.get_joint_state()}
    
# for test
if __name__ == "__main__":

    b = (0, 0.0, 0.0) 
    a = (0,1,4)
    abc = FashionstarRobot(a,b,"dev/ttyUSB0",1000000)

