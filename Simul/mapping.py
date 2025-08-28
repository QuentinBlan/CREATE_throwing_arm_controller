import numpy as np
import matplotlib.pyplot as plt

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import your forward kinematics
from kinematic import Kinematic  # Change 'your_module_name' to your file name

# --- Robot parameters ---
params = {
    'L': [0.3542, 0.289, 0.2268],               # section lengths [m]
    'nPoints': 10,                       # points per section
    'r_disk': [0.0449, 0.0376, 0.0270],      # tendon routing radius [m]
    'sigma': np.array([[0, 2*np.pi/3, 4*np.pi/3],
                        [np.pi/6, 5*np.pi/6, 3*np.pi/2],
                        [np.pi/3, np.pi, 5*np.pi/3]]),  # tendon angles [rad]
    'Kbend': [17.860, 11.819, 7.036],       # bending stiffness [N·m²]
}

# --- Simulation setup ---
n_tendons = 9 # cannot be changed
tension_min = 0.0
tension_max = 200.0
n_steps = 40 # number of steps per tendon
actuated_tendons = [6,7,8]

kinematic = Kinematic()

# Workspace storage: list for each tendon
workspace_points = [[] for _ in range(n_tendons)]

# Sweep tensions
for tendon_idx in actuated_tendons:
    for tension_value in np.linspace(tension_min, tension_max, n_steps):
        tensions = np.zeros(n_tendons)
        tensions[tendon_idx] = tension_value  # actuate only this tendon
        positions, frames, curvatures = kinematic.soft_arm_pcc(tensions)

        # Tip position = last column of last frame
        tip_pos = positions[:, -1]
        workspace_points[tendon_idx].append(tip_pos)

# --- Plot ---
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

colors = plt.cm.tab10(np.linspace(0, 1, n_tendons))

for i in actuated_tendons:
    points = np.array(workspace_points[i])
    ax.plot(points[:, 0], points[:, 1], points[:, 2], '-o', color=colors[i], label=f"Tendon {i+1}")

ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
ax.set_title("Workspace Mapping (Single Tendon Actuation)")
ax.legend()
ax.grid(True)
plt.tight_layout()
plt.show()

