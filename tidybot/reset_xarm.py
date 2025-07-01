from xarm.wrapper import XArmAPI
import time

arm = XArmAPI('192.168.1.203')

arm.clean_error()
arm.clean_warn()
arm.motion_enable(True)
arm.set_mode(0)          # position
arm.set_state(0)         # ready
time.sleep(0.1)          # let brakes release

code = arm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0, 0],wait=True)   # block until finished
if code != 0:
    print('ret code:', code)
else:
    print("*"*30,"GOT HOME","*"*30)
arm.disconnect()
