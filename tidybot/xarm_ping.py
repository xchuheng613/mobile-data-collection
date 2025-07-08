from xarm.wrapper import XArmAPI
import time

arm = XArmAPI('192.168.1.203')

arm.clean_error(); arm.clean_warn()
arm.motion_enable(True)
arm.set_mode(0)          # position
arm.set_state(0)         # ready
time.sleep(0.3)          # let brakes release

err = arm.set_servo_angle(
        angle=[0, 0, 0, 0, 0, 0, 0],
        speed=40,
        is_radian=False,
        wait=False)   # block until finished
print('ret code:', err)
arm.disconnect()
