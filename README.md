# FABRIK_Soft
# 3D Soft Robot FABRIK Simulation

This repository contains a Python implementation of a 3D soft robot simulation that uses a modified FABRIK (Forward And Backward Reaching Inverse Kinematics) algorithm to solve the inverse kinematics problem for a soft robotic arm. The simulation integrates simple physics (gravity and damping) and provides interactive sliders to adjust the target position in real time.

## Features

- **3D Visualization:**  
  The simulation uses Matplotlib to display the soft robot in a 3D space.

- **Modified FABRIK Algorithm:**  
  The inverse kinematics is solved using an iterative forward and backward reaching approach. The algorithm allows for flexible segment lengths (80% to 120% of the nominal value) to mimic the elastic behavior of soft robots.

- **Physics Integration:**  
  Gravity and damping are applied to the robot's joints to simulate realistic motion.

- **Interactive Controls:**  
  Three sliders (X, Y, and Z) let you dynamically change the target position for the end-effector. The robot updates its configuration in real time.

## Requirements

- Python 3.x

- [NumPy](https://numpy.org/)
- [Matplotlib](https://matplotlib.org/)

```bash
pip install numpy matplotlib
