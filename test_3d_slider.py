import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D, proj3d
from matplotlib.widgets import Slider
import matplotlib.animation as animation

# ---------------------------
# Soft Robot 3D Class
# ---------------------------
class SoftRobot3D:
    def __init__(self, base, segment_lengths):
        self.base = np.array(base, dtype=float)
        self.segment_lengths = np.array(segment_lengths, dtype=float)
        self.num_segments = len(segment_lengths)
        self.points = self._initialize_points()
        self.velocities = np.zeros_like(self.points)
        # Physics parameters
        self.damping = 0.98
        # Gravity (scaled for simulation speed)
        self.gravity = np.array([0, -9.81, 0], dtype=float) * 0.1
        # Stretch limits: allow 80% to 120% of nominal length
        self.min_factor = 0.8
        self.max_factor = 1.2

    def _initialize_points(self):
        # Create a chain starting at the base along the x-axis
        points = [self.base.copy()]
        for length in self.segment_lengths:
            points.append(points[-1] + np.array([length, 0, 0]))
        return np.array(points)

    def apply_physics(self, dt):
        # Update velocities (skip the fixed base) with gravity and apply damping
        for i in range(1, self.num_segments + 1):
            self.velocities[i] += self.gravity * dt
        self.points[1:] += self.velocities[1:] * dt
        self.velocities *= self.damping
        self.points[0] = self.base  # Ensure base remains fixed

    def solve_constraints(self, target, iterations=5):
        """
        A FABRIK-style constraint projection.
        The end effector is forced toward the target,
        then backward and forward passes adjust the segments while keeping the
        segment lengths within allowed stretch/contraction limits.
        """
        target = np.array(target, dtype=float)
        # Set the end effector to the target
        self.points[-1] = target.copy()
        # Iteratively apply the projection
        for _ in range(iterations):
            # Backward pass: from end effector back to the base
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

            # Forward pass: from the base to the end effector
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

# ---------------------------
# Global Simulation Setup
# ---------------------------
base_position = [0, 0, 0]
segment_lengths = [1] * 10  # 10 segments of unit length
robot = SoftRobot3D(base_position, segment_lengths)
# Initial target (x, y, z)
target = np.array([5, 5, 5], dtype=float)

# ---------------------------
# Set Up the 3D Plot and Sliders
# ---------------------------
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# Adjust bottom space for sliders
fig.subplots_adjust(bottom=0.3)

# Create initial plot objects for the robot and markers
xs, ys, zs = robot.points[:, 0], robot.points[:, 1], robot.points[:, 2]
line, = ax.plot(xs, ys, zs, 'bo-', label="Soft Robot")
base_scatter = ax.scatter([robot.base[0]], [robot.base[1]], [robot.base[2]], 
                          c='red', s=100, label="Base")
target_scatter = ax.scatter([target[0]], [target[1]], [target[2]], 
                            c='green', s=100, label="Target")
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_zlim(-10, 10)
ax.set_title("3D Soft Robot with Slider-Controlled Target")
ax.legend()

# Create slider axes for X, Y, and Z
ax_slider_x = fig.add_axes([0.2, 0.18, 0.65, 0.03])
ax_slider_y = fig.add_axes([0.2, 0.12, 0.65, 0.03])
ax_slider_z = fig.add_axes([0.2, 0.06, 0.65, 0.03])

slider_x = Slider(ax_slider_x, 'X', -10, 10, valinit=target[0])
slider_y = Slider(ax_slider_y, 'Y', -10, 10, valinit=target[1])
slider_z = Slider(ax_slider_z, 'Z', -10, 10, valinit=target[2])

def update_target(val):
    # Update the global target based on slider values
    target[0] = slider_x.val
    target[1] = slider_y.val
    target[2] = slider_z.val

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
    line.set_data(xs, ys)
    line.set_3d_properties(zs)
    # Update target scatter marker
    target_scatter._offsets3d = ([target[0]], [target[1]], [target[2]])
    return line, target_scatter

# Create a persistent animation object
ani = animation.FuncAnimation(fig, animate, interval=50, cache_frame_data=False, save_count=200)

plt.show()
