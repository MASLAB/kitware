# Kitware

ROS2 example code for the kitbot (MASLAB 2022)

* Controls 2 Kitbot drive motors with WASD keys
* Visualize data with rqt perspective

## Setup
1. Clone into the `src` directory of your colcon workspace for ROS
2. Run `colcon build` from the workspace folder
3. Make sure you have the `pygame` package: `pip3 install pygame`
4. Make sure you have TAMProxy-Firmware running on a teensy 
5. Add the TAMProxy-pyHost folder to your `PYTHONPATH`

## Running

### Using rosrun
* Launch the kitbot node: `ros2 run kitware kitbot.py`
* Launch the keyboard driver node: `ros2 run kitware kbd_driver.py`

### Using roslaunch
* `ros2 launch kitware kitware_launch.py`

### Using rqt
* Launch rqt: `rqt`
* From the perspectives menu, select "import perspective" and select the `kitware.perspective` file in the root of this repo.
* At the bottom, click the ROS Launch GUI. Start both the `kitbot.py` and `kbd_driver.py` nodes
