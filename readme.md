# Kitware

ROS2 example code for the kitbot (MASLAB 2024)

* Controls 2 Kitbot drive motors with WASD keys
* Visualize data with rqt perspective

## Setup
1. Clone into the `src` directory of your colcon workspace for ROS
2. Make sure you have cloned [kitware_interface](https://github.com/MASLAB/kitware_interface) in `src` as well
3. Run `colcon build` from the workspace folder

The following steps should already be done on the hardware we give students.
1. Make sure you have the `pygame` package: `pip3 install pygame`
2. Make sure you have TAMProxy-Firmware running on a teensy
3. Add the TAMProxy-pyHost folder to your `PYTHONPATH`

## Running

Run `. install/setup.bash` to setup your environment to use the built packages

### Using rosrun
* Launch the kitbot node: `ros2 run kitware kitbot`
* Launch the keyboard driver node: `ros2 run kitware kbd_driver`

### Using roslaunch
* `ros2 launch kitware kitware_launch.py`

## Using rqt
* Launch both the kitbot and keyboard driver nodes.
* Launch rqt: `rqt`
* From the perspectives menu, select "import perspective" and select the `kitware.perspective` file in the root of this repo.
* Here you can visualize the values sent in the `/drive_cmd` topic and see a graph of the nodes and topics.
