# Teleoperation 
## Base Control
There are three files related to base control:
- `base_server.py`
- `base_test.py`
- `base_controller.py`

The files test the validation of the Franka base.  
 `base_server.py` defines the base manager and start a server.  
 `base_test.py` sends command to perform certain trajectory.  
 `base_controller.py` defines the controller class and handle the hardware response.

To test the files, run the following commands in separate terminals:  
```sh
python3 base_server.py
python3 base_test.py
```

## Arm Control
There are three files related to arem control:
- `arm_server.py`
- `arm_test.py`
- `arm_controller.py`
- `ik_solver.py`

The files test the validation of the Xarm7.  
 `arm_server.py` defines the arm manager and start a server.  
 `arm_test.py` sends command to perform certain trajectory.  
 `arm_controller.py` defines the controller class and handle the hardware response.
 `ik_solver.py` defines a inverse kinematic solver for Xarm7.

To test the files, run the following commands in separate terminals:  
```sh
python3 arm_server.py
python3 arm_test.py
```

## Keyboard Base Control
`keyboard_teleop.py`  
This file allows you to control the base through keyboard.  
```sh
python3 keyboard_teleop.py
```

## Teleoperate Simulation
To perform a teleoperate simulation through mujoco, execute
```sh
python3 main.py --sim --teleop
```
You need to have a second device to connect the server (192.168.0.X:5000 in our case). After you connected to the server, press `Start episode` to start the simulation. Pressing on the edge which the screen will turn red, you can control the base movement. To control the arm, press on the middle of your screen which it will turn blue.

## Teleoperate in Real
To perform teleoperation in reality, you need to run three files simultaniously.
```sh
python3 arm_server.py
python3 base_server.py
python3 main.py --teleop
```
Similar to the previous simulation, you need a second device to connect to the server and control the robot.