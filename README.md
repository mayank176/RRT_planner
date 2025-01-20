## Sampling-Based Motion Planning for Quadrotors in Cluttered Environments 

This repository provides a comprehensive motion planning and control framework in C++ designed for quadrotor navigation in complex environments with obstacles. The system integrates three key components:

1. Path Planning: Implements the Rapidly-exploring Random Tree (RRT) algorithm, a probabilistic sampling-based approach that efficiently explores the configuration space to find collision-free paths. The generated paths are post-processed using Bézier curves to ensure smooth trajectories that respect the quadrotor's dynamics.

2. Trajectory Control: Offers two controller implementations:
   - A nonlinear PD controller 
   - A model predictive control (MPC) based controller that optimizes the trajectory by considering future states

3. Visualization & Integration: Built on ROS2 middleware with full visualization support through rviz2, where we visualize:

   - the RRT exploration tree
   - Smoothed Bézier trajectories
   - Controller performance metrics
   - Obstacle map and collision boundaries

## System Requirements

ROS2, C++17 or later, Eigen3, Casadi, rviz2

## Installation

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install foobar.

```bash
pip install f
```

## Configuration Options

1. Choose an environment:

<p align="center">
  <img src="docs/media/env1.gif" width="30%" />
  &nbsp;&nbsp;&nbsp;
  <img src="docs/media/env2.gif" width="30%" />
  &nbsp;&nbsp;&nbsp;
  <img src="docs/media/env3.gif" width="30%" />
</p>
<p align="center">
  <em>Env 1</em>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <em>Env 2</em>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <em>Env 3</em>
</p>


To create a custom environment, change/add obstacles in rrt_vis/src/rrt_vis.cpp

2. RRT Parameters:
   - step_size: control the granularity of the RRT tree expansion (default 0.75 m)
   - max_iterations: Maximum number of iterations for pathfinding (default: 10000) 

3. Controller Parameters
   - control_frequency: Controller update rate (default:100 Hz)
   - N: Prediction horizon for MPC (default: 20 steps)
   - K,K,K,K : PD gains for lateral and altitude control


## Usage

After configuring, simply run the following launch file to start the RRT planner, controller, and visualization in rviz2:

```C++
ros2 launch rrt_vis rrt_planner.launch.py
```

## Troubleshooting

Common issues and solutions:
1. Rviz2 visualization: Ensure correct path to rviz config file in launch file


## License

[MIT](https://choosealicense.com/licenses/mit/)