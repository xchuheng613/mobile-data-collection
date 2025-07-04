import numpy as np
import time
from ruckig import Ruckig, InputParameter, OutputParameter, Result

# Parameters
DOF = 7
CONTROL_PERIOD = 0.001  # 1ms, 1000Hz control loop

# Create OTG instance
otg = Ruckig(DOF, CONTROL_PERIOD)
inp = InputParameter(DOF)
out = OutputParameter(DOF)

# Initial state (robot is at zero and not moving)
inp.current_position = np.zeros(DOF)
inp.current_velocity = np.zeros(DOF)
inp.current_acceleration = np.zeros(DOF)

# Target state
inp.target_position = np.array([0.5, -0.3, 0.6, -0.5, 0.2, -0.4, 0.1])
inp.target_velocity = np.zeros(DOF)
inp.target_acceleration = np.zeros(DOF)

# Velocity and acceleration limits
inp.max_velocity = np.array([6.0, 6.0, 6.0, 6.0, 6.0, 8.0, 8.0])
inp.max_acceleration = np.array([12.0, 12.0, 12.0, 12.0, 12.0, 16.0, 16.0])

print("Starting OTG demo...")
step = 0
while True:
    result = otg.update(inp, out)
    if result != Result.Working:
        print("OTG complete:", result)
        break

    print(f"Step {step:03d} | q: {[round(x, 3) for x in out.new_position]}")
    step += 1

    out.pass_to_input(inp)
    time.sleep(CONTROL_PERIOD)
