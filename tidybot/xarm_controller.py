#!/usr/bin/env python3
"""
arm_controller_xarm.py  – Compliant, jerk-limited joint controller
Streams **angles** to xArm-7 via xarm_shim.TorqueControlledArm
"""
import math, time, numpy as np, queue, threading
from ruckig import Ruckig, InputParameter, OutputParameter, Result
from constants import POLICY_CONTROL_PERIOD
from xarm_shim import TorqueControlledArm

# gains (unchanged)
ALPHA = 0.01
K_r  = np.diag([0.3]*4 + [0.18]*3)
K_l  = np.diag([75.0]*4 + [40.0]*3)
K_lp = np.diag([5.0]*4 + [4.0]*3)
K_p  = np.diag([100.0]*4 + [50.0]*3)
K_d  = np.diag([3.0]*4 + [2.0]*3)
K_r_inv = np.linalg.inv(K_r)
K_r_K_l = K_r @ K_l
DT = 0.004       # 250 Hz

class LowPass:
    def __init__(self, a, y0): self.a, self.y = a, y0
    def __call__(self, x): self.y = self.a*x+(1-self.a)*self.y; return self.y

class JointCompliantController:
    def __init__(self, cmd_q): self.cmd_q = cmd_q; self._init = False
    # ------------------------------------------------------
    def _lazy(self, arm):
        self.q_s = self.q_d = self.q_n = arm.q.copy()
        self.dq_d = self.dq_n = np.zeros(7)
        self.tau_f = LowPass(ALPHA, arm.tau.copy())
        self.last = time.time()

        self.otg = Ruckig(7, DT)
        self.inp = InputParameter(7); self.out = OutputParameter(7)
        self.inp.max_velocity     = 4*[math.radians(80)] + 3*[math.radians(140)]
        self.inp.max_acceleration = 4*[math.radians(240)] + 3*[math.radians(450)]
        self.inp.current_position = self.inp.target_position = arm.q.copy()
        self.inp.current_velocity = [0.0]*7
        self.res = Result.Finished
        self._init = True
    # ------------------------------------------------------
    def __call__(self, arm: TorqueControlledArm):
        if not self._init: self._lazy(arm)

        self.q_s = self.q_s + np.mod(arm.q - self.q_s + np.pi, 2*np.pi) - np.pi
        dq_s = arm.dq.copy(); tau_s = self.tau_f(arm.tau)

        # new goal?
        if not self.cmd_q.empty():
            q, _ = self.cmd_q.get_nowait(); self.last = time.time()
            q = self.q_s + np.mod(q - self.q_s + np.pi, 2*np.pi) - np.pi
            self.inp.target_position = q; self.res = Result.Working

        if time.time() - self.last > 2.5*POLICY_CONTROL_PERIOD:
            self.inp.target_position = self.out.new_position; self.res = Result.Working

        if self.res == Result.Working:
            self.res = self.otg.update(self.inp, self.out)
            self.out.pass_to_input(self.inp)
            self.q_d, self.dq_d = self.out.new_position, self.out.new_velocity

        # compliant plant integrated in angle space
        g = arm.gravity()
        tau = -K_p@(self.q_n-self.q_d) - K_d@(self.dq_n-self.dq_d) + g
        ddq = K_r_inv@(tau - tau_s); self.dq_n += ddq*DT; self.q_n += self.dq_n*DT
        return self.q_n.copy(), 0.0      # <- joint angles only

# ------------------------------------------------------------------
def _demo_cmd_loop(cmd_q, stop_evt):
    q_retract = np.deg2rad([0, -20, 180, -146, 0, -50, 90])
    while not stop_evt.is_set():
        cmd_q.put((q_retract, 0.0)); time.sleep(POLICY_CONTROL_PERIOD)

if __name__ == "__main__":
    arm = TorqueControlledArm(inner_dt=DT)
    q = queue.Queue(maxsize=1)
    ctrl = JointCompliantController(q)
    stop = threading.Event()
    threading.Thread(target=_demo_cmd_loop, args=(q, stop), daemon=True).start()
    arm.init_cyclic(ctrl)
    try:  
        while True: time.sleep(1)
    except KeyboardInterrupt: stop.set(); arm.stop_cyclic()
