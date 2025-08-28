import numpy as np
import sys
import os


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from kinematic import Kinematic



def test_trans_mat_cc(kinematic: Kinematic):
    print("=== Testing trans_mat_cc ===")
    kappa = 0.5      # curvature [1/m]
    phi = np.pi / 4  # bending plane angle [rad]
    L = 1.0          # arc length [m]
    n_pts = int(5)
    
    T = kinematic.trans_mat_cc(kappa, phi, L, n_pts)

    print("Shape:", T.shape)  # Expect (5, 4, 4)
    print("First transform:\n", T[0])
    print("Last transform:\n", T[-1])
    assert T.shape == (n_pts, 4, 4), "Unexpected shape from trans_mat_cc"
    assert np.allclose(T[:, 3, 3], 1), "Homogeneous coordinates not correct"
    print("trans_mat_cc test passed!\n")


def test_soft_arm_pcc(kinematic: Kinematic):
    print("=== Testing soft_arm_pcc ===")

    tensions = np.array([5.0, 4.0, 6.0,
                         3.0, 2.0, 4.0,
                         2.0, 1.0, 3.0])    # [N]

    positions, frames, curvatures = kinematic.soft_arm_pcc(tensions)

    print("Positions shape:", positions.shape)
    print("Frames[0] shape:", frames[0].shape)
    print("Curvatures:\n", curvatures)
    assert positions.shape[1] == kinematic.params['nPoints'] * len(kinematic.params['L']), "Positions shape mismatch"
    assert all(f.shape == (kinematic.params['nPoints'], 4, 4) for f in frames), "Frame shape mismatch"
    print("soft_arm_pcc test passed!\n")

def test_inverse_kinematic(kinematic: Kinematic):
    target_point = (0.1, 0.05, 0.85)  # some arbitrary desired point
    (t1, t2, t3), (x_found, y_found, z_found), error = kinematic.inverse_kinematic(target_point)

    print("\nTarget position:", target_point)
    print("Nearest map position:", (x_found, y_found, z_found))
    print("Required tensions:  T1={:.2f}, T2={:.2f}, T3={:.2f}".format(t1, t2, t3))
    print("Position error: {:.4f} m".format(error))
    print("test passed")

def test_generate_new_workspace(kinematic : Kinematic):
    kinematic.generate_lookup_table()


if __name__ == "__main__":
    kinematic = Kinematic()
    test_trans_mat_cc(kinematic)
    test_soft_arm_pcc(kinematic)
    test_inverse_kinematic(kinematic)
    #test_generate_new_workspace(kinematic)
