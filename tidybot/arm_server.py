
import time
from multiprocessing.managers import BaseManager
from arm_controller import Xarm, XArm7Config
from constants import ARM_RPC_HOST, ARM_RPC_PORT, RPC_AUTHKEY


class Arm:
    def __init__(self):
        self.arm = None

    def reset(self):
        if self.arm is not None:
            if self.arm.control_loop_running:
                self.arm.stop_control()
        print("="*30,"RESETTING","="*30)
        xarm7_cfg = XArm7Config()
        self.arm = Xarm(xarm7_cfg)
        
        # Start low-level control
        self.arm.start_control()
        while not self.arm.control_loop_running:
            time.sleep(0.01)

    def execute_action(self, action): 
        self.arm.set_target_state(action)

    def get_state(self):
        state = {'arm_pos': self.arm.ee_pos, 'arm_euler': self.arm.ee_rpy, 'arm_quat': self.arm.ee_quat, 'arm_qpos': self.arm.q} 
        return state

    def close(self):
        if self.arm is not None:
            if self.arm.control_loop_running:
                self.arm.stop_control()

class ArmManager(BaseManager):
    pass

ArmManager.register('Arm', Arm)

arm_manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
server = arm_manager.get_server()
print(f'Arm manager server started at {ARM_RPC_HOST}:{ARM_RPC_PORT}')
server.serve_forever()

























import queue
import time
from multiprocessing.managers import BaseManager as MPBaseManager
import numpy as np
from arm_controller import JointCompliantController
from constants import ARM_RPC_HOST, ARM_RPC_PORT, RPC_AUTHKEY
from kinova import TorqueControlledArm

class Arm:
    def __init__(self, max_vel=10, max_accel=10, is_radian=False):
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.arm = None
        self.is_radian=is_radian

    def reset(self):
        self.arm = TorqueControlledArm()
        self.arm.set_joint_limits(speed_limit=self.max_vel, acceleration_limit=self.max_accel, is_radian=self.is_radian)
        print("Arm Resetted!")
        
        self.command_queue = queue.Queue(1)
        self.controller = JointCompliantController(self.command_queue)
        
        self.arm.start_cyclic(self.controller.control_callback)
        while not self.arm.cyclic_running:
            time.sleep(0.01)

    def execute_action(self, action):
        position = np.concatenate([
            action['arm_pos'],
            action['arm_euler']
        ]).tolist()
        print(self.arm.arm.get_position()[1])
        qpos = self.arm.arm.get_inverse_kinematics(position, input_is_radian=False, return_is_radian=True)
        print(f"ik result: {qpos[1]}")
        self.command_queue.put((qpos, action['gripper_pos'].item())) 
        
    def get_state(self):
        arm_position = self.arm.arm.get_position(is_radian=True)[1]
        state = {
            'arm_pos': arm_position[:3],
            'arm_euler': arm_position[3:],
            'gripper_pos': np.array([self.arm.gripper_pos]),
        }
        return state

    def close(self):
        if self.arm.cyclic_running:
            time.sleep(0.75)  # Wait for arm to stop moving
            self.arm.stop_cyclic()
        self.arm.disconnect()

class ArmManager(MPBaseManager):
    pass

ArmManager.register('Arm', Arm)

if __name__ == '__main__':
    manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    server = manager.get_server()
    print(f'Arm manager server started at {ARM_RPC_HOST}:{ARM_RPC_PORT}')
    server.serve_forever()