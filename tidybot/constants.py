import numpy as np

################################################################################
# Vehicle center to steer axis (m)
h_x, h_y = 0.190150 * np.array([1.0, 1.0, -1.0, -1.0]), 0.170150 * np.array([-1.0, 1.0, 1.0, -1.0])  # Kinova / Franka

# Encoder magnet offsets
ENCODER_MAGNET_OFFSETS = [-2023.0 / 4096, -1450.0 / 4096, -416.0 / 4096, 1170.0 / 4096]
################################################################################
# Base and arm RPC servers
BASE_RPC_HOST = 'localhost'
BASE_RPC_PORT = 10000
ARM_RPC_HOST = 'localhost'
ARM_RPC_PORT = 10001
RPC_AUTHKEY = b'secret password'

# Cameras
BASE_CAMERA_SERIAL = '6F3E2ADE'
WRIST_CAMERA_SERIAL = 'E469C3EE' 

# Policy
POLICY_SERVER_HOST = 'localhost'
POLICY_SERVER_PORT = 5555
POLICY_CONTROL_FREQ = 10
POLICY_CONTROL_PERIOD = 1.0 / POLICY_CONTROL_FREQ
POLICY_IMAGE_WIDTH = 84
POLICY_IMAGE_HEIGHT = 84