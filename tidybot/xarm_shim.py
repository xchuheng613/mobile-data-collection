"""
xarm_shim.py  ·  Minimal position-mode wrapper for an xArm-7
"""

import time, threading, numpy as np
from pathlib import Path
import pinocchio as pin
from xarm.wrapper import XArmAPI

# ─────────────────────────────────────────────────────────────
class TorqueControlledArm:          # keep old class name for imports
    def __init__(self,
                 arm_ip: str = "192.168.1.203",
                 inner_dt: float = 0.004):
        self.inner_dt = inner_dt
        self.actuator_count = 7

        # Pinocchio model (for gravity only)
        urdf = Path(__file__).parent / "models/ufactory_xarm7/xarm7.urdf"
        self.pin_model = pin.buildModelFromUrdf(str(urdf))
        self.pin_data  = self.pin_model.createData()
        self.ee_frame  = [f.name for f in self.pin_model.frames].index("link_eef")

        # SDK
        self.arm = XArmAPI(arm_ip)
        self.arm.connect()
        self.arm.motion_enable()
        self.arm.set_mode(1)       # servo-joint
        self.arm.set_state(0)

        # live state
        self.q  = np.zeros(7)
        self.dq = np.zeros(7)
        self.tau = np.zeros(7)     # no torque feedback
        self.gripper_pos = 0.0

        # cyclic sender
        self._cmd   = np.zeros(7)
        self._lock  = threading.Lock()
        self._alive = True
        threading.Thread(target=self._tx_loop, daemon=True).start()

    # ---------------------------------------------------------
    def _tx_loop(self):
        while self._alive:
            time.sleep(self.inner_dt)
            with self._lock:
                q_cmd = self._cmd.copy()
            self.arm.set_servo_angle_j(q_cmd, speed=120, mvacc=3000,
                                       wait=False, is_radian=True)
            code, js = self.arm.get_joint_states(is_radian=True)
            if code == 0:
                self.q, self.dq = np.asarray(js[0]), np.asarray(js[1])

    # ---------------------------------------------------------
    # methods expected by higher layers
    def set_joint_limits(self, speed_limits=None, acceleration_limits=None):
        if speed_limits is not None:
            self.max_vel = speed_limits
        if acceleration_limits is not None:
            self.arm.set_joint_maxacc(acceleration_limits, is_radian=False)

    def open_gripper(self):  pass        # no gripper on baseline xArm-7
    def retract(self):       pass        # stub – implement if you need it
    def clear_faults(self):
        self.arm.clean_error(); self.arm.clean_warn()

    def gravity(self):
        return pin.computeGeneralizedGravity(self.pin_model,
                                             self.pin_data, self.q)

    def get_tool_pose(self):
        """
        Returns (xyz [m], quat [x y z w]) – never raises.
        Strategy:
        1. Try firmware up to 3× (6 ms total)
        2. If still busy, compute FK from the latest joint angles
        """
        import time, math, numpy as np, pinocchio as pin

        # Helper: quick FK
        def _fk(q):
            pin.forwardKinematics(self.pin_model, self.pin_data, q)
            pin.updateFramePlacements(self.pin_model, self.pin_data)
            oMe = self.pin_data.oMf[self.ee_frame]
            xyz  = oMe.translation
            quat = pin.Quaternion(oMe.rotation).coeffs()      # (w xyz)
            return xyz, np.array([quat[1], quat[2], quat[3], quat[0]])

        # ①  up to 3 attempts
        for _ in range(3):
            code, pose = self.arm.get_position(is_radian=True)
            if code == 0:                                     # success
                xyz = np.asarray(pose[:3]) / 1000.0           # mm → m
                r, p, y = pose[3:]
                cy, sy = math.cos(y/2), math.sin(y/2)
                cp, sp = math.cos(p/2), math.sin(p/2)
                cr, sr = math.cos(r/2), math.sin(r/2)
                qw = cr*cp*cy + sr*sp*sy
                qx = sr*cp*cy - cr*sp*sy
                qy = cr*sp*cy + sr*cp*sy
                qz = cr*cp*sy - sr*sp*cy
                return xyz, np.array([qx, qy, qz, qw])
            time.sleep(0.002)                                 # 2 ms

        # ②  firmware still busy → FK fallback
        return _fk(self.q)

    # ---------------------------------------------------------
    # cyclic glue used in arm_controller
    def control_arm_qpos(self, q_des):
        with self._lock:
            self._cmd = np.asarray(q_des)

    def init_cyclic(self, cb):
        def _loop():
            while self._alive:
                q_cmd, _ = cb(self)
                self.control_arm_qpos(q_cmd)
                time.sleep(self.inner_dt)
        threading.Thread(target=_loop, daemon=True).start()

    def stop_cyclic(self):
        self._alive = False
        time.sleep(0.1)
        self.arm.set_state(4)
        self.arm.disconnect()

    # property used by server loop
    @property
    def cyclic_running(self):
        return self._alive
