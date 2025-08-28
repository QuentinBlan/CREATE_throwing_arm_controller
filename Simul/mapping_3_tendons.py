import numpy as np
import matplotlib.pyplot as plt
import sys
import os
from scipy.spatial import cKDTree  # fast nearest neighbor search

# Import forward kinematics
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from kinematic import Kinematic

kinematic = Kinematic()
kinematic.generate_lookup_table()

# --- Example usage ---
target_point = (0.1, 0.05, 0.4)  # some arbitrary desired point
(t1, t2, t3), (x_found, y_found, z_found), error = kinematic.inverse_kinematic(target_point)
workspace_map = kinematic.lookup_table

print("\nTarget position:", target_point)
print("Nearest map position:", (x_found, y_found, z_found))
print("Required tensions:  T1={:.2f}, T2={:.2f}, T3={:.2f}".format(t1, t2, t3))
print("Position error: {:.4f} m".format(error))

# --- Plot workspace ---
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(workspace_map[:, 0], workspace_map[:, 1], workspace_map[:, 2],
           s=5, alpha=0.5, label="Workspace")
ax.scatter(*target_point, color='r', s=80, label="Target")
ax.scatter(x_found, y_found, z_found, color='g', s=80, label="Nearest")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
ax.legend()
ax.set_title("Workspace & Nearest Point Lookup")
plt.tight_layout()
plt.show()
