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

How to Run
Clone the repository:

bash
Copy
git clone https://github.com/yourusername/soft-robot-fabrik.git
cd soft-robot-fabrik
Run the simulation:

bash
Copy
python your_script_name.py
This will open a 3D window with interactive sliders at the bottom. Adjust the sliders to change the target position of the robot's end-effector.

Code Structure
SoftRobot3D Class:
Implements the 3D robot simulation, including physics integration (gravity and damping) and the FABRIK-based constraint solver.

Interactive Widgets:
Uses Matplotlib's widgets to create sliders that control the X, Y, and Z coordinates of the target.

Animation Loop:
The simulation is updated continuously using Matplotlib's FuncAnimation, which smoothly updates the robot's configuration.

The FABRIK Algorithm
The FABRIK (Forward And Backward Reaching Inverse Kinematics) algorithm is used to iteratively adjust the positions of a series of joints to reach a target point while respecting distance constraints.

Suppose we have a chain of joints:

𝑝
0
,
𝑝
1
,
…
,
𝑝
𝑛
p 
0
​
 ,p 
1
​
 ,…,p 
n
​
 
with nominal segment lengths:

𝐿
0
,
𝐿
1
,
…
,
𝐿
𝑛
−
1
L 
0
​
 ,L 
1
​
 ,…,L 
n−1
​
 
and a target position for the end-effector 
𝑡
t.

Forward Reaching Phase
Set the end effector to the target:

𝑝
𝑛
=
𝑡
p 
n
​
 =t
Iterate from the end effector back to the base:
For 
𝑖
=
𝑛
−
1
,
𝑛
−
2
,
…
,
0
i=n−1,n−2,…,0:

Compute the current distance:
𝑟
=
∥
𝑝
𝑖
+
1
−
𝑝
𝑖
∥
r=∥p 
i+1
​
 −p 
i
​
 ∥
Compute the scaling factor:
𝜆
=
𝐿
𝑖
𝑟
λ= 
r
L 
i
​
 
​
 
Update the position:
𝑝
𝑖
=
𝑝
𝑖
+
1
+
𝜆
(
𝑝
𝑖
−
𝑝
𝑖
+
1
)
p 
i
​
 =p 
i+1
​
 +λ(p 
i
​
 −p 
i+1
​
 )
Backward Reaching Phase
Fix the base at its original position:

𝑝
0
=
base
p 
0
​
 =base
Iterate from the base to the end effector:
For 
𝑖
=
0
,
1
,
…
,
𝑛
−
1
i=0,1,…,n−1:

Compute the current distance:
𝑟
=
∥
𝑝
𝑖
+
1
−
𝑝
𝑖
∥
r=∥p 
i+1
​
 −p 
i
​
 ∥
Compute the scaling factor:
𝜆
=
𝐿
𝑖
𝑟
λ= 
r
L 
i
​
 
​
 
Update the position:
𝑝
𝑖
+
1
=
𝑝
𝑖
+
𝜆
(
𝑝
𝑖
+
1
−
𝑝
𝑖
)
p 
i+1
​
 =p 
i
​
 +λ(p 
i+1
​
 −p 
i
​
 )
Note: In our implementation, the segment lengths are allowed to vary between 80% and 120% of the nominal length to simulate soft (flexible) behavior.

Physics Integration
Before applying the FABRIK solver, a simple physics update is performed:

Gravity: A constant gravitational acceleration is applied to each joint (except the fixed base).
Damping: Velocities are damped to produce smooth, gradual motion.
Contributing
Contributions, bug reports, and suggestions are welcome! Please feel free to open an issue or submit a pull request.

License
This project is licensed under the MIT License.

Install dependencies using pip:

```bash
pip install numpy matplotlib
