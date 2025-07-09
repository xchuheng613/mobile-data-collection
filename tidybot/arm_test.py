import time
from multiprocessing.managers import BaseManager
from constants import ARM_RPC_HOST, ARM_RPC_PORT, RPC_AUTHKEY
import numpy as np
from constants import POLICY_CONTROL_PERIOD

class ArmManager(BaseManager):
    pass

ArmManager.register('Arm')

if __name__ == '__main__':
    arm_manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    arm_manager.connect()
    arm = arm_manager.Arm()

    arm_pos = np.array([343, 449, 330]) / 1000
    arm_euler = np.deg2rad(np.array([-131, 6.8, 33.9]))
    gripper_pos = np.zeros(1)

    try:
        arm.reset()
        while True:
            time.sleep(POLICY_CONTROL_PERIOD) 
            arm.execute_action({'arm_pos': arm_pos,'arm_euler': arm_euler,'gripper_pos': gripper_pos})
            err = np.linalg.norm(arm_pos - arm.get_state()['arm_pos'])
            print(f"q: {[round(x, 3) for x in arm.get_state()['arm_pos']]}, euler:{[round(x,3) for x in np.rad2deg(arm.get_state()['arm_euler'])]}")

    finally:
        arm.close()
