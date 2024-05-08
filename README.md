# PID Maze Solver

The PID Maze Solver is a ROS2 package designed to control a Rosbot XL robot to navigate through a maze using two PID controllers. The first controller moves the robot forward toward a waypoint, and the second controller rotates the robot to face the next waypoint. This solution integrates both controllers to allow the Rosbot XL to solve mazes efficiently.

## Features

- Two finely-tuned PID controllers for forward movement and rotation.
- Switching between simulation (Gazebo) and real-world (CyberLab) environments.
- Flexible waypoint management using YAML configuration files.
- Seamless alternation between turning and moving forward states.

## Dependencies

- **ROS2**
- **geometry_msgs**: Message types for geometry data
- **nav_msgs**: Message types for navigation data
- **yaml-cpp**: YAML configuration file management

## Installation

1. Clone the repository into your ROS2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/MiguelSolisSegura/pid_maze_solver.git
    ```

2. Install all required dependencies:
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. Build the workspace:
    ```bash
    colcon build
    source install/setup.bash
    ```

## Usage

1. **Simulated Maze:**

    To launch the maze in Gazebo and run the PID solver:
    ```bash
    ros2 launch rosbot_xl_gazebo simulation.launch.py
    ros2 run pid_maze_solver pid_maze_solver 1
    ```

2. **Real Maze (CyberLab):**

    To use the real robot in CyberLab:
    ```bash
    ros2 run pid_maze_solver pid_maze_solver 2
    ```

