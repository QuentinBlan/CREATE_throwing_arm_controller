import numpy as np
from scipy.spatial import cKDTree  # fast nearest neighbor search

class Kinematic:
    
    def __init__(self):


        self.params = {
            'L': [0.3542, 0.289, 0.2268],  # section lengths [m]
            'nPoints': 10,                 # points per section
            'r_disk': [0.0449, 0.0376, 0.0270],  # tendon routing radius [m]
            'sigma': np.array([[0, 2*np.pi/3, 4*np.pi/3],
                               [np.pi/6, 5*np.pi/6, 3*np.pi/2],
                               [np.pi/3, np.pi, 5*np.pi/3]]),
            'Kbend': [17.860, 11.819, 7.036],# bending stiffness [N·m²]
          }  

        try:
            self.lookup_table = np.load("workspace_map.npy")
        except Exception as e:
            print(f'could not open the lookup table: {e} \n generating a default one')
            self.generate_lookup_table()


        positions_only = self.lookup_table[:, :3]  # just X, Y, Z
        self.tree = cKDTree(positions_only)

        #option to generate a new one ? 


    def inverse_kinematic(self, target_xyz):
        """Given (x, y, z), return nearest tensions (t1, t2, t3)."""
        dist, idx = self.tree.query(target_xyz)
        x, y, z, t1, t2, t3 = self.lookup_table[idx]

        return (t1, t2, t3), (x, y, z), dist
    
    def generate_lookup_table(self) -> None:
        # --- Simulation setup ---
        n_tendons = 9
        tension_min = 0.0
        tension_max = 200.0
        n_steps = 25  # increase for more accuracy
        actuated_tendons = [6, 7, 8]  # indices for the tendons to control

        tension_values = np.linspace(tension_min, tension_max, n_steps)

        # --- Storage: each row = [x, y, z, t1, t2, t3] ---
        workspace_map = []

        print(f"Simulating {n_steps**3} combinations...")
        i = 0
        for t1 in tension_values:
            for t2 in tension_values:
                for t3 in tension_values:
                    tensions = np.zeros(n_tendons)
                    tensions[actuated_tendons[0]] = t1
                    tensions[actuated_tendons[1]] = t2
                    tensions[actuated_tendons[2]] = t3

                    positions, frames, curvatures = self.soft_arm_pcc(tensions)
                    tip_pos = positions[:, -1]  # end-effector position

                    workspace_map.append([tip_pos[0], tip_pos[1], tip_pos[2], t1, t2, t3])
            i += 1
            print(f"Progress: {int(i*n_steps**2)} / {n_steps**3}")

        self.lookup_table = np.array(workspace_map)

        # --- Save map ---
        np.save("workspace_map.npy", self.lookup_table)
        np.savetxt("workspace_map.csv", self.lookup_table, delimiter=",",
                header="X,Y,Z,T1,T2,T3", comments='')
        print("Workspace map saved.")

        # --- Build KDTree for fast lookup ---
        positions_only = self.lookup_table[:, :3]  # just X, Y, Z
        self.tree = cKDTree(positions_only)
            
    
    def soft_arm_pcc(self, tensions):
        """
        Compute static and kinematic shape of a 3-section tendon-driven soft arm.

        Parameters
        ----------
        tensions : (9,) array_like
            Tendon tensions [N]
        params : dict
            Required keys:
                'L'      : (3,) section lengths [m]
                'nPoints': int, number of sample points per section
                'r_disk' : (3,) tendon routing radius for each section [m]
                'sigma'  : (3,3) angular positions of tendons per section [rad]
                'Kbend'  : (3,) bending stiffness (E·I) [N·m²]

        Returns
        -------
        positions : (3, 3*nPoints) array
            3D coordinates along the backbone
        frames : list of 3 arrays
            Each array is (nPoints, 4, 4) homogeneous transforms for a section
        curvatures : (3, 3) array
            Rows = [kappa_j, phi_j, L_j] for each section
        """
        # Extract parameters
        L       = np.asarray(self.params['L'], dtype=float).reshape(-1)
        n       = int(self.params['nPoints'])
        r_disk  = np.asarray(self.params['r_disk'], dtype=float).reshape(-1)
        sigma   = np.asarray(self.params['sigma'], dtype=float)
        Kbend   = np.asarray(self.params['Kbend'], dtype=float).reshape(-1)

        n_sec = len(L)

        tensions = np.asarray(tensions, dtype=float).reshape(-1)

        # Step 1: Local bending moments (vectorized over tendons)
        Mx_local = r_disk * np.sum(tensions.reshape(n_sec, 3) * np.cos(sigma), axis=1)
        My_local = r_disk * np.sum(tensions.reshape(n_sec, 3) * np.sin(sigma), axis=1)

        # Step 2: Cumulative moments
        Mx_cum = np.array([np.sum(Mx_local[j:]) for j in range(n_sec)])
        My_cum = np.array([np.sum(My_local[j:]) for j in range(n_sec)])

        # Step 3: Curvature params
        phi_vals = np.arctan2(My_cum, Mx_cum)
        M_mags   = np.hypot(Mx_cum, My_cum)
        kappa_vals = M_mags / Kbend
        curvatures = np.vstack((kappa_vals, phi_vals, L))

        # Step 4: Forward kinematics
        frames = []
        positions = np.zeros((3, n_sec * n))
        T_prev = np.eye(4)

        for j in range(n_sec):
            kappa, phi, lj = curvatures[:, j]
            Tj_local = self.trans_mat_cc(kappa, phi, lj, n)

            # Global transforms: batch multiply T_prev with all local transforms
            Tj_global = T_prev @ Tj_local

            frames.append(Tj_global)

            # Store positions (batch extract last column)
            positions[:, j*n:(j+1)*n] = Tj_global[:, :3, 3].T

            # Update for next section
            T_prev = Tj_global[-1]

        return positions, frames, curvatures
    

    def trans_mat_cc(self, kappa, phi, L, n_pts, tol=1e-6):
        """
        Generate constant-curvature homogeneous transforms.
        
        Parameters
        ----------
        kappa : float
            Curvature (1 / radius of curvature)
        phi : float
            Bending-plane angle in radians
        L : float
            Total arc length
        n_pts : int
            Number of sample points along the arc
        tol : float, optional
            Threshold for treating curvature as zero
        
        Returns
        -------
        T : ndarray of shape (n_pts, 4, 4)
            Homogeneous transforms from base to each sample point
        """

        s_vals = np.linspace(0, L, n_pts)

        # Precompute rotation into/out of bending plane
        cphi, sphi = np.cos(phi), np.sin(phi)
        Rz1 = np.array([[ cphi, -sphi, 0],
                        [ sphi,  cphi, 0],
                        [ 0,     0,    1]])
        Rz2 = np.array([[ cphi,  sphi, 0],
                        [-sphi,  cphi, 0],
                        [ 0,     0,    1]])

        if abs(kappa) < tol:
            # Straight segment
            R_all = np.repeat(np.eye(3)[None, :, :], n_pts, axis=0)
            p_all = np.column_stack((np.zeros(n_pts), np.zeros(n_pts), s_vals))
        else:
            theta_vals = kappa * s_vals
            ct = np.cos(theta_vals)
            st = np.sin(theta_vals)

            # Batch rotation about local Y
            Ry_all = np.zeros((n_pts, 3, 3))
            Ry_all[:, 0, 0] = ct
            Ry_all[:, 0, 2] = st
            Ry_all[:, 1, 1] = 1
            Ry_all[:, 2, 0] = -st
            Ry_all[:, 2, 2] = ct

            # Apply Rz1 @ Ry @ Rz2 in batch
            R_all = Rz1 @ Ry_all @ Rz2

            # Positions
            p_all = (1.0 / kappa) * np.column_stack([
                (1 - ct) * cphi,
                (1 - ct) * sphi,
                st
            ])

        # Build homogeneous transforms
        T = np.zeros((n_pts, 4, 4))
        T[:, :3, :3] = R_all
        T[:, :3, 3] = p_all
        T[:, 3, 3] = 1.0

        return T


    if __name__ == "__main__":
        kappa = 0.5      # 1/m
        phi = np.pi/4    # 45°
        L = 1.0          # meters
        n_pts = 5

        T = trans_mat_cc(kappa, phi, L, n_pts)
        print("Shape:", T.shape)       # (5, 4, 4)
        print("First transform:\n", T[4])
