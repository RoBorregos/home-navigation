# DOCUMENTATION

## Overview

This document provides detailed documentation for each ROS node used in the Home Navigation project. It includes explanations of their purposes, the technologies used, and examples of how to use them.

## ROS Nodes

### actions

#### moveServer.py

**Purpose**: This node is responsible for handling navigation actions, such as moving the robot to a specified location, moving forward, moving backward, and detecting door signals.

**Technologies Used**: 
- ROS ActionLib
- MoveBase
- RANSAC Regressor

**Example Usage**:
```bash
roslaunch actions move_actions.launch
```

#### moveClient.py

**Purpose**: This node is responsible for sending navigation goals to the moveServer node.

**Technologies Used**: 
- ROS ActionLib

**Example Usage**:
```bash
roslaunch actions move_actions.launch
```

### dashgo_driver

#### driver_imu.launch

**Purpose**: This launch file starts the serial communication with the STM32 microcontroller and initializes the IMU driver.

**Technologies Used**: 
- ROS Serial
- IMU

**Example Usage**:
```bash
roslaunch dashgo_driver driver_imu.launch
```

### ydlidar

#### ydlidar1_up.launch

**Purpose**: This launch file starts the Lidar sensor.

**Technologies Used**: 
- ROS Lidar

**Example Usage**:
```bash
roslaunch ydlidar ydlidar1_up.launch
```

### nav_main

#### robot_navigation.launch

**Purpose**: This launch file starts the navigation stack, including the map server, AMCL, and MoveBase.

**Technologies Used**: 
- ROS Navigation Stack
- AMCL
- MoveBase

**Example Usage**:
```bash
roslaunch nav_main robot_navigation.launch
```

## Examples

### Basic Setup

To set up the environment, including building the Docker image and creating a container, follow these steps:

1. Build the Docker image:
```bash
make nav.build
```

2. Create a container:
```bash
make nav.create
```

### Running the Robot

To start the robot and its components, follow these steps:

1. Start the Lidar:
```bash
roslaunch ydlidar ydlidar1_up.launch
```

2. Start the serial STM32:
```bash
roslaunch dashgo_driver driver_imu.launch
```

### Using ROS Nodes

To use the ROS nodes and packages, follow these steps:

1. Start the moveServer and moveClient nodes:
```bash
roslaunch actions move_actions.launch
```

### Launch Files

To use the launch files, follow these steps:

1. Start the navigation stack:
```bash
roslaunch nav_main robot_navigation.launch
```

### Configuration Files

To modify and use the configuration files, follow these steps:

1. Edit the configuration file:
```bash
nano ws/src/dashgo/dashgo_driver/config/my_dashgo_params.yaml
```

2. Save the changes and restart the relevant nodes.

### Scripts

To use the scripts, follow these steps:

1. Run the check_linear_imu.py script:
```bash
rosrun dashgo_tools check_linear_imu.py
```

### Maps and Navigation

To use the maps and navigation features, follow these steps:

1. Start the map server:
```bash
roslaunch nav_main robot_navigation.launch
```

### Contributing

To contribute to the project, follow these steps:

1. Fork the repository on GitHub.
2. Clone your forked repository to your local machine.
3. Create a new branch for your changes.
4. Make your changes and commit them with clear and concise commit messages.
5. Push your changes to your forked repository.
6. Create a pull request to the main repository, describing your changes and the purpose of the contribution.

### License

This project is licensed under the GNU General Public License v3.0. See the LICENSE file for more details.
