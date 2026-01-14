# MATLAB-Simulation-Of-Obstacle-Avoiding-Robot-With-Potential-Field-Path-Planning-
# SIMULATION-OF-OBSTACLE-AVOIDING-ROBOT-WITH-POTENTIAL-FIELD-PATH-PLANNING
This project simulates autonomous robot path planning in a 2D environment using the artificial potential field method, enabling real-time obstacle avoidance with both static and dynamic obstacles and interactive visualization in MATLAB.
## Objectives
- Implement artificial potential field–based navigation
- Avoid collisions with static and moving obstacles
- Demonstrate real-time robot motion in a dynamic environment
- Visualize robot trajectory, obstacle influence zones, and goal convergence

## Algorithm Used
- The robot’s motion is governed by:
- Attractive force pulling the robot toward the goal
- Repulsive forces pushing the robot away from nearby obstacles
- A safety-check mechanism to prevent collisions
A small random perturbation is added to help escape local minima.

## Algorithm Used
- The robot’s motion is governed by:
- Attractive force pulling the robot toward the goal
- Repulsive forces pushing the robot away from nearby obstacles
- A safety-check mechanism to prevent collisions
- A small random perturbation is added to help escape local minima.

How to Run
Open MATLAB
Run the script:
```bash
obstacle_avoiding1.m
