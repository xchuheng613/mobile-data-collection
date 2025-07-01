#!/usr/bin/env python3
"""
arm_server_xarm.py  ·  RPC wrapper exposing reset/execute/get_state for xArm-7
"""

import queue, time, numpy as np
from multiprocessing.managers import BaseManager

from constants      import ARM_RPC_HOST, ARM_RPC_PORT, RPC_AUTHKEY
from xarm_shim      import TorqueControlledArm
from xarm_controller import JointCompliantController
from ik_solver_xarm import IKSolver

class Arm:
    def __init__(self, ip="192.168.1.203"):
        self.arm = TorqueControlledArm(arm_ip=ip)

        # speed limits are useful; accel limits can stay stock
        self.arm.set_joint_limits(speed_limits=[0.8]*4 + [1.0, 1.0, 1.5])

        self.cmd_q = queue.Queue(maxsize=1)
        self.ctrl  = None
        self.ik    = IKSolver()


    # ----------------------------------------------------------
    def reset(self):
        if self.arm.cyclic_running:
            self.arm.stop_cyclic()
        self.arm.clear_faults()

        self.ctrl = JointCompliantController(self.cmd_q)
        # 🔑 pass the *method*, not the whole object
        self.arm.init_cyclic(self.ctrl)

        # wait until the thread is confirmed running
        while not self.arm.cyclic_running:
            time.sleep(0.02)

    # ----------------------------------------------------------
    def execute_action(self, action: dict) -> bool:
        """Return True if queued, False if pose unreachable."""
        try:
            q_goal = self.ik.solve(action['arm_pos'],
                                   action['arm_quat'],
                                   self.arm.q)          # seed = live joints
        except RuntimeError as e:
            if "code 10" in str(e):
                print("[Arm] pose unreachable – skipped")
                return False
            raise                                            # other errors

        # overwrite 1-slot queue non-blocking
        try:
            self.cmd_q.get_nowait()                          # drop stale goal
        except queue.Empty:
            pass
        
        self.cmd_q.put_nowait((q_goal, float(action['gripper_pos'])))
        return True

    # ----------------------------------------------------------
    def get_state(self):
        pos, quat = self.arm.get_tool_pose()                 # never raises
        if quat[3] < 0: quat = -quat                         # unique quat
        return {'arm_pos': pos,
                'arm_quat': quat,
                'gripper_pos': np.array([self.arm.gripper_pos])}

    # ----------------------------------------------------------
    def close(self):
        if self.arm.cyclic_running:
            time.sleep(0.5)
            self.arm.stop_cyclic()

# --------------------------------------------------------------------------
class ArmManager(BaseManager): pass
ArmManager.register('Arm', Arm)

if __name__ == "__main__":
    mgr = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT),
                     authkey=RPC_AUTHKEY)
    print(f"Arm RPC server running at {ARM_RPC_HOST}:{ARM_RPC_PORT}")
    mgr.get_server().serve_forever()
