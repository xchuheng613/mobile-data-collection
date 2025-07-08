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
        state = {'arm_pos': self.arm.ee_pos, 'arm_euler': self.arm.ee_rpy, 'arm_quat': self.arm.ee_quat, 'arm_qpos': self.arm.q, 'gripper_pos': self.arm.gripper_pos} 
        return state

    def close(self):
        if self.arm is not None:
            if self.arm.control_loop_running:
                self.arm.stop_control()

class ArmManager(BaseManager):
    pass

ArmManager.register('Arm', Arm)


if __name__ == '__main__':
    arm_manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    server = arm_manager.get_server()
    print(f'Arm manager server started at {ARM_RPC_HOST}:{ARM_RPC_PORT}')
    server.serve_forever()
