# Kitware

ROS example code for the kitbot (MASLAB 2020)

* Controls 2 Kitbot drive motors with WASD keys
* Visualize data with rqt perspective

## Setup
1. Clone into the `src` directory of your Catkin workspace for ROS
2. Run `catkin_make` from the workspace folder
3. Make sure you have the `pygame` package: `pip install pygame`

## Running

### Step 0: Ensure roscore is running
`roscore`

### Using rosrun
* Launch the kitbot node: `rosrun kitware kitbot.py`
* Launch the keyboard driver node: `rosrun kitware kbd_driver.py`

### Using roslaunch
* `roslaunch kitware kitware.launch`

### Using rqt
* Launch rqt: `rqt`
* From the perspectives menu, select "import perspective" and select the `kitware.perspective` file in the root of this repo.
* At the bottom, click the ROS Launch GUI. Start both the `kitbot.py` and `kbd_driver.py` nodes
