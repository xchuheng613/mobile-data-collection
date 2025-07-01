import math
import queue
import threading
import time
import numpy as np
import subprocess
from xarm.wrapper import XArmAPI

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._prev_err = None
        self._cum_err = 0

    def reset(self):
        self._prev_err = None
        self._cum_err = 0

    def control(self, err, dt):
        if self._prev_err is None:
            self._prev_err = err

        value = (
                self.kp * err
                + self.kd * (err - self._prev_err) / dt
                + self.ki * self._cum_err
        )

        self._prev_err = err
        self._cum_err += dt * err
        return value
    
    
class XArm7Config():
    name:               str = "xArm7"
    dof:                int = 7
    enable:             bool = True
    arm_ip:             str = "192.168.1.203"
    use_servo_control:  bool = False
    use_gripper:        bool = False
    ctrl_freq :         int = 250
    ctrl_period :       float = 1.0 / ctrl_freq

class Xarm:
    def __init__(self, config: XArm7Config):
        try:
            subprocess.run(['ping', '-c', '1',  config.arm_ip], check=True, timeout=1, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("*"*20,"PINGING ARM","*"*20)
        except subprocess.TimeoutExpired as e:
            raise Exception('Could not communicate with arm') from e
        
        self.dof = config.dof
        self.enable = config.enable
        self.use_servo_control = config.use_servo_control
        self.use_gripper = config.use_gripper
        self.CTRL_FREQ = config.ctrl_freq
        self.CTRL_PERIOD = config.ctrl_period
        
        
        self.q = np.zeros(self.dof)
        self.dq = np.zeros(self.dof)
        self.tau = np.zeros(self.dof)
        
        self.ee_pos = np.zeros(3)
        self.ee_quat = np.zeros(4)
        self.ee_rpy = np.zeros(3)
        
        self.arm = XArmAPI(port=config.arm_ip)
        self.reset()
        
        if not self.use_servo_control:
            default_kp = np.array([2, 2, 1, 1, 1, 1, 1]) * 5
            default_kd = default_kp / 20
            default_ki = np.zeros(7)
            self.max_arm_velocity = np.array(
                [math.radians(180*0.5)] * 4 + [math.radians(360*0.5)] * 3
                # [0.8, 0.8, 0.8, 0.8, 1.0, 1.0, 1.5]
                # [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
            )
            self.arm_pid = PIDController(
                kp=default_kp,
                ki=default_ki,
                kd=default_kd,
            )
        else:
            self.arm_velocity_limit = 3.0
        
        self.command_queue = queue.Queue(1)
        self.control_loop_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_loop_lock = threading.Lock()
        self.control_loop_running = False
        

    def reset(self):
        print("*"*20,"RESETTING ARM","*"*20)
        self.xarm_mode = 1 if self.use_servo_control else 4

        self.arm.emergency_stop()
        self.arm.clean_error()
        self.arm.motion_enable(True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(0.1)
        if self.use_gripper:
            self.arm.set_gripper_mode(0)
            self.arm.set_gripper_enable(True)
            self.arm.set_gripper_position(850, wait=True)
            self.gripper_pos = 1.0
        self.arm.move_gohome(wait=False)
        while True:
            code, current_q = self.arm.get_servo_angle(is_radian=True)
            if code != 0:
                print("Failed to get joint angles.")
                break

            err = np.abs(np.array(current_q) - np.array([0,0,0,0,0,0,0]))
            if np.all(err < 1e-3):
                break
            time.sleep(0.1)
        
        self.arm.set_mode(self.xarm_mode)
        self.arm.set_state(0)
        time.sleep(0.1)
        print("*"*20,"GOT HOME","*"*20)
            

        
    def set_target_state(self, action):
        state = np.concatenate([
            action['arm_pos'],
            action['arm_euler']
        ])
        code, qpos = self.arm.get_inverse_kinematics(state, input_is_radian=False, return_is_radian=True) 
        print("="*20,f"ik code: {code}", "="*20)
        print(f"ik result: {[round(x,4) for x in qpos]}")
        self._enqueue_command((qpos, action['gripper_pos']))

    def _enqueue_command(self, target):
        if self.command_queue.full():
            print('Warning: Command queue is full. Is control loop running?')
        elif not self.enable:
            print("Not Enabled")
        else:
            with self.control_loop_lock:
                self.command_queue.put(target, block=False)
            
    def control_loop(self):
        command = None
        while self.control_loop_running:
            self.update_state()
            time.sleep(self.CTRL_PERIOD)
            with self.control_loop_lock:
                if not self.command_queue.empty() or command is None:
                    command = self.command_queue.get()
                    print("-"*10,"GOT NEW COMMAND","-"*10)
                    
                code, _ = self.arm.get_state()
                if code != 0:
                    print("*" * 50)
                    print(f"Arm error: {code}")
                    print("*" * 50)
                    self.enable = False

                if self.use_servo_control:
                    safe_control_qpos = self.clip_arm_next_qpos(
                        command[0], velocity_limit=self.arm_velocity_limit
                    )
                    self.arm.set_servo_angle_j(angles=safe_control_qpos)
                else:
                    code, xarm_state = self.arm.get_joint_states(is_radian=True)
                    arm_current_qpos = np.array(xarm_state[0])
                    error = np.array(command[0]) - arm_current_qpos
                    qvel = self.arm_pid.control(error, self.CTRL_PERIOD)
                    safe_qvel = self.clip_arm_velocity(qvel)
                    code = self.arm.vc_set_joint_velocity(safe_qvel)
                
    def clip_arm_next_qpos(self, target_qpos, velocity_limit=0.8):
        code, cur_arm_state = self.arm.get_joint_states(is_radian=True)
        current_qpos = cur_arm_state[0]
        error = target_qpos - current_qpos
        motion_scale = np.max(np.abs(error)) / (velocity_limit * self.CTRL_PERIOD)
        safe_control_qpos = current_qpos + error / motion_scale
        return safe_control_qpos
    
    def clip_arm_velocity(self, arm_qvel: np.ndarray):
        velocity_overshot = np.abs(arm_qvel) / self.max_arm_velocity
        max_overshot = np.max(velocity_overshot)
        if max_overshot > 1 + 1e-4:
            safe_velocity = arm_qvel / max_overshot
            bottleneck_joint = np.argmax(velocity_overshot)
            print(f"Bottleneck joint for velocity clip: joint-{bottleneck_joint + 1} with overshoot {max_overshot}")
        else:
            safe_velocity = arm_qvel
        return safe_velocity

    def start_control(self):
        if self.control_loop_thread is None:
            print('To initiate a new control loop, please create a new instance of Vehicle.')
            return
        self.control_loop_running = True
        self.control_loop_thread.start()

    def stop_control(self):
        self.control_loop_running = False
        self.arm.vc_set_joint_velocity(np.zeros(7))
        self.control_loop_thread.join()
        self.control_loop_thread = None
        
    def update_state(self):
        def _rpy_to_quat(rpy):
            import math
            roll, pitch, yaw = rpy
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            q = np.array([
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy
            ])
            return q
        code, state = self.arm.get_joint_states()
        self.q = state[0]
        self.dq = state[1]
        self.tau = state[2]
        if self.use_gripper:
            code, self.gripper_pos = self.arm.get_gripper_position() / 850
            
                
        code, ee_state = self.arm.get_position()
        self.ee_pos = ee_state[:3]
        self.ee_rpy = ee_state[3:]
        self.ee_quat = _rpy_to_quat(self.ee_rpy)
        