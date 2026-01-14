# MATLAB-Simulation-Of-Obstacle-Avoiding-Robot-With-Potential-Field-Path-Planning-
# SIMULATION-OF-OBSTACLE-AVOIDING-ROBOT-WITH-POTENTIAL-FIELD-PATH-PLANNING
This project simulates autonomous robot navigation in a 2D environment using the Artificial Potential Field (APF) method. The robot is capable of real-time obstacle avoidance for both static and dynamic obstacles, with interactive visualization implemented in MATLAB.

## Objectives
- Implement artificial potential field–based navigation
- Avoid collisions with static and moving obstacles
- Demonstrate real-time robot motion in a dynamic environment
- Visualize robot trajectory, obstacle influence zones, and goal convergence

## Algorithm Used
The robot’s motion is governed by:
- Attractive Force – Pulls the robot toward the goal position.
- Repulsive Force – Pushes the robot away from nearby obstacles to avoid collisions.
- Safety Check – Ensures the robot does not collide with obstacles.
- Random Perturbation – Small random adjustments help the robot escape local minima in the potential field.
A small random perturbation is added to help escape local minima.

## Features
- Handles static and moving obstacles.
- Interactive MATLAB visualization of the robot, obstacles, and goal.
- Real-time trajectory updates showing robot path planning.
- Demonstrates how potential field methods can avoid local minima issues.

## Usage
- Open the MATLAB project folder.
- Run the main script (main.m or equivalent).
- Observe the robot navigating through the environment while avoiding obstacles.
- Adjust parameters such as obstacle positions, goal location, and force constants to test different scenarios.

## Requirements
- MATLAB R2020a or later.
- MATLAB toolboxes for plotting and visualization (e.g., MATLAB Graphics).


How to Run
Open MATLAB
Run the script:
```bash
obstacle_avoiding1.m
