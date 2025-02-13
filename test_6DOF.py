import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D, proj3d
from matplotlib.widgets import Slider
import matplotlib.animation as animation

# ---------------------------
# Soft Robot 3D Class for a 6DOF Robot Arm
# ---------------------------
class SoftRobot3D:
    def __init__(self, base, segment_lengths):
        self.base = np.array(base, dtype=float)
        self.segment_lengths = np.array(segment_lengths, dtype=float)
        self.num_segments = len(segment_lengths)  # For 6DOF: 6 segments => 7 joints
        self.points = self._initialize_points()
        self.velocities = np.zeros_like(self.points)
        self.damping = 0.98
        # Gravity vector (scaled for simulation speed)
        self.gravity = np.array([0, -9.81, 0], dtype=float) * 0.1
        # Stretch limits (allow 80% to 120% of the nominal segment length)
        self.min_factor = 0.8
        self.max_factor = 1.2

    def _initialize_points(self):
        # Initialize a straight configuration along the x-axis
        points = [self.base.copy()]
        for length in self.segment_lengths:
            points.append(points[-1] + np.array([length, 0, 0]))
        return np.array(points)

    def apply_physics(self, dt):
        # Update velocities (for all joints except the fixed base) with gravity
        for i in range(1, self.num_segments + 1):
            self.velocities[i] += self.gravity * dt
        self.points[1:] += self.velocities[1:] * dt
        self.velocities *= self.damping
        self.points[0] = self.base  # Ensure the base stays fixed

    def solve_constraints(self, target, iterations=5):
        """
        FABRIK-style iterative solver:
          - Force the end effector (last joint) to the target.
          - Perform a backward pass to adjust joints from the end toward the base.
          - Perform a forward pass from the base back to the end.
          - Clamp each segment's length within [0.8L, 1.2L].
        """
        target = np.array(target, dtype=float)
        self.points[-1] = target.copy()
        for _ in range(iterations):
            # Backward Reaching
            for i in range(self.num_segments, 0, -1):
                r = np.linalg.norm(self.points[i] - self.points[i-1])
                if r == 0:
                    continue
                L_nominal = self.segment_lengths[i-1]
                L_min = L_nominal * self.min_factor
                L_max = L_nominal * self.max_factor
                lam = L_nominal / r
                if r < L_min:
                    lam = L_min / r
                elif r > L_max:
                    lam = L_max / r
                self.points[i-1] = self.points[i] + (self.points[i-1] - self.points[i]) * lam
            # Forward Reaching
            self.points[0] = self.base
            for i in range(self.num_segments):
                r = np.linalg.norm(self.points[i+1] - self.points[i])
                if r == 0:
                    continue
                L_nominal = self.segment_lengths[i]
                L_min = L_nominal * self.min_factor
                L_max = L_nominal * self.max_factor
                lam = L_nominal / r
                if r < L_min:
                    lam = L_min / r
                elif r > L_max:
                    lam = L_max / r
                self.points[i+1] = self.points[i] + (self.points[i+1] - self.points[i]) * lam

    def update(self, target, dt):
        self.apply_physics(dt)
        self.solve_constraints(target, iterations=5)

def compute_joint_angles(points):
    """
    Compute bending angles (in degrees) between consecutive segments.
    For a 6DOF robot (7 points), returns 5 angles.
    """
    angles = []
    for i in range(1, len(points)-1):
        v1 = points[i] - points[i-1]
        v2 = points[i+1] - points[i]
        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)
        if norm1 == 0 or norm2 == 0:
            angle = 0
        else:
            dot = np.dot(v1, v2)
            cos_angle = np.clip(dot / (norm1 * norm2), -1, 1)
            angle = np.arccos(cos_angle)
        angles.append(angle * 180 / np.pi)
    return angles

# ---------------------------
# Global Simulation Setup
# ---------------------------
base_position = [0, 0, 0]
segment_lengths = [1, 1, 1, 1, 1, 1]  # 6 segments for a 6DOF arm
robot = SoftRobot3D(base_position, segment_lengths)
target = np.array([5, 5, 5], dtype=float)

# ---------------------------
# Set Up the 3D Plot and Slider Widgets
# ---------------------------
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# Adjust the bottom to make space for sliders
fig.subplots_adjust(bottom=0.25)

ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_zlim(-10, 10)
ax.set_title("6DOF Robot Arm with FABRIK, Joint Angles, and Slider Target")

# Plot objects: the robot arm line, target marker, and joint angle text.
line, = ax.plot(robot.points[:, 0], robot.points[:, 1], robot.points[:, 2],
                'bo-', label="Robot Arm")
target_scatter = ax.scatter([target[0]], [target[1]], [target[2]],
                            c='green', s=100, label="Target")
angle_text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes,
                       fontsize=10, color='blue')

# Create slider axes for target X, Y, and Z coordinates.
ax_slider_x = fig.add_axes([0.15, 0.10, 0.65, 0.03])
ax_slider_y = fig.add_axes([0.15, 0.05, 0.65, 0.03])
ax_slider_z = fig.add_axes([0.15, 0.00, 0.65, 0.03])

slider_x = Slider(ax_slider_x, 'Target X', -10, 10, valinit=target[0])
slider_y = Slider(ax_slider_y, 'Target Y', -10, 10, valinit=target[1])
slider_z = Slider(ax_slider_z, 'Target Z', -10, 10, valinit=target[2])

def update_target(val):
    target[0] = slider_x.val
    target[1] = slider_y.val
    target[2] = slider_z.val

# Connect the slider update events.
slider_x.on_changed(update_target)
slider_y.on_changed(update_target)
slider_z.on_changed(update_target)

# ---------------------------
# Animation Update Function
# ---------------------------
def animate(frame):
    dt = 0.05
    robot.update(target, dt)
    xs, ys, zs = robot.points[:, 0], robot.points[:, 1], robot.points[:, 2]
    # Update the robot arm line.
    line.set_data(xs, ys)
    line.set_3d_properties(zs)
    # Update the target marker.
    target_scatter._offsets3d = ([target[0]], [target[1]], [target[2]])
    # Compute joint angles between segments.
    angles = compute_joint_angles(robot.points)
    text_str = "Joint Angles (deg): " + ", ".join(["{:.1f}".format(a) for a in angles])
    angle_text.set_text(text_str)
    return line, target_scatter, angle_text

ani = animation.FuncAnimation(fig, animate, interval=50, cache_frame_data=False, save_count=200)

plt.legend()
plt.show()
