# Home Navigation

## Overview and Introduction

The Home Navigation project aims to develop a robust and reliable set of software modules tailored to the specific needs of a service robot. The project focuses on creating a functional robot for both the Mexican Robotics Tournament and the RoboCup. The robot, named FRIDA (Friendly Robotic Interactive Domestic Assistant), is designed to assist with various tasks in a domestic environment.

## Achievements and Objectives

### Achievements from 2024

This year, after being accepted to participate in RoboCup 2024 held in Eindhoven, Netherlands, the team focused on developing a robust and reliable set of software modules, tailored to the specific needs for the tasks for the updated rulebook. This approach had the purpose of showcasing a functional robot for both the Mexican Robotics Tournament (April; Monterrey, Mexico) and the RoboCup (July; Eindhoven, Netherlands).

The robot was renamed as FRIDA (Friendly Robotic Interactive Domestic Assistant), an acronym reflecting the purpose of the robot, and the name in reference to Mexican culture.

With the vast knowledge acquired during the international tournament, the team defined the new objectives for the remainder of the year to be: an increased focus on research and literature review, and centralized and offline refactorization of the software and systems.

## Directory Structure

The following is the directory structure of the project, along with explanations for each directory and its purpose:

```
.
├── docker
│   ├── Dockerfile.nav
│   ├── Dockerfile.nav.mediapipe
│   ├── Dockerfile.nav.xavier
│   └── scripts
│       └── run.bash
├── ws
│   ├── src
│   │   ├── actions
│   │   │   ├── action
│   │   │   ├── launch
│   │   │   ├── scripts
│   │   │   └── srv
│   │   ├── dashgob1_noetic
│   │   │   ├── src
│   │   │   │   ├── dashgo
│   │   │   │   │   ├── dashgo_description
│   │   │   │   │   ├── dashgo_driver
│   │   │   │   │   ├── dashgo_nav
│   │   │   │   │   ├── dashgo_rviz
│   │   │   │   │   └── dashgo_tools
│   │   │   └── notes.md
│   │   ├── map_contextualizer
│   │   │   ├── contextmaps
│   │   │   ├── include
│   │   │   ├── msg
│   │   │   ├── scripts
│   │   │   └── srv
│   │   ├── nav_main
│   │   │   ├── config
│   │   │   ├── filters
│   │   │   ├── launch
│   │   │   ├── maps
│   │   │   ├── rviz
│   │   │   └── scripts
│   │   ├── robot_pose_publisher
│   │   │   ├── include
│   │   │   └── src
│   │   ├── ydlidar_v1.3.1
│   │   │   ├── launch
│   │   │   ├── sdk
│   │   │   └── src
│   │   └── yocs_velocity_smoother
│   │       ├── include
│   │       ├── launch
│   │       ├── param
│   │       └── src
│   └── .catkin_workspace
├── .gitignore
├── .gitmodules
├── LICENSE
├── Makefile
└── README.md
```

## Makefile Commands

The Makefile contains the following commands:

- `nav.build`: Builds the Docker image
- `nav.create`: Creates a container based on the image previously built, attaches the ws folder, and sets up hostnames and IPs necessary for multimaster
- `nav.up`: Starts the container
- `nav.down`: Stops the container
- `nav.shell`: Starts a shell session inside the container
- `nav.remove`: Removes the container

## Contributing

To contribute to the project, follow these steps:

1. Fork the repository on GitHub.
2. Clone your forked repository to your local machine.
3. Create a new branch for your changes.
4. Make your changes and commit them with clear and concise commit messages.
5. Push your changes to your forked repository.
6. Create a pull request to the main repository, describing your changes and the purpose of the contribution.

## License

This project is licensed under the GNU General Public License v3.0. See the LICENSE file for more details.
