import casadi as ca
from scipy.spatial.transform import Rotation as R
import numpy as np
import dartpy as dart
from casadi import MX, DM
import matplotlib.pyplot as plt

def rotation_vector_difference(rotvec_a, rotvec_b):
    R_a = R.from_rotvec(rotvec_a)
    R_b = R.from_rotvec(rotvec_b)
    R_diff = R_b.inv() * R_a
    return R_diff.as_rotvec()

def pose_difference(pose_a, pose_b):
    pos_diff = pose_a[:3] - pose_b[:3]
    rot_diff = rotation_vector_difference(pose_a[3:], pose_b[3:])
    return np.hstack((pos_diff, rot_diff))

# converts a rotation matrix to a rotation vector
def get_rotvec(rot_matrix):
    rotation = R.from_matrix(rot_matrix)
    return rotation.as_rotvec()

def block_diag(*arrays):
    arrays = [np.atleast_2d(a) if np.isscalar(a) else np.atleast_2d(a) for a in arrays]

    rows = sum(arr.shape[0] for arr in arrays)
    cols = sum(arr.shape[1] for arr in arrays)
    block_matrix = np.zeros((rows, cols), dtype=arrays[0].dtype)

    current_row = 0
    current_col = 0

    for arr in arrays:
        r, c = arr.shape
        block_matrix[current_row:current_row + r, current_col:current_col + c] = arr
        current_row += r
        current_col += c

    return block_matrix


def log_status(lite3, time, to_file):
    state = lite3.retrieve_state()

    status = {
        'FL_FOOT': {'pos': state['FL_FOOT']['pos'][3:]},
        'HL_FOOT': {'pos': state['HL_FOOT']['pos'][3:]},
        'FR_FOOT': {'pos': state['FR_FOOT']['pos'][3:]},
        'HR_FOOT': {'pos': state['HR_FOOT']['pos'][3:]},
        'com': {'pos': state['com']['pos']}
    }

    log_text = "\n=== STATUS ===\n"
    log_text += f"Current time: {np.round(time, 3)}\n"  # Aggiungi il time corrente

    for key, value in status.items():
        pos = value['pos']
        log_text += f"{key} position: {np.round(pos, 3)}\n"

    step_index = lite3.footstep_planner.get_step_index_at_time(time)
    start_time = lite3.footstep_planner.get_start_time(step_index) if step_index is not None else None
    current_phase = lite3.footstep_planner.get_phase_at_time(time) if step_index is not None else None

    log_text += "\n=== PLAN INFO ===\n"
    log_text += f"Current step index: {step_index}\n"
    log_text += f"Start time of current step: {start_time}\n"
    log_text += f"Current feet target: {current_phase}\n"
    log_text += "==================\n"

    if to_file:
        with open('log_status.txt', 'a') as f:
            f.write(log_text) 
        print(f"Current time: {np.round(time, 3)}\n")
        print("Log scritto su file 'log_status.txt'.")
    else:
        print(log_text)


def log_mpc(mpc_class, t, x_des_num, swing_inverted, forces):
    if not hasattr(mpc_class, "first_write"):
      mpc_class.first_write = True

    if mpc_class.first_write:
        mode = "w"
        mpc_class.first_write = False
    else:
        mode = "a"  

    labels = [
        "roll ", "pitch", "yaw  ",
        "CoM_x", "CoM_y", "CoM_z",
        "ang_x", "ang_y", "ang_z",
        "lin_x", "lin_y", "lin_z",
        "grav "
    ]

    with open("x_des_log.txt", mode) as file:
        file.write(f"Step t = {t}\n")
        file.write("x_des_num:\n")
        
        for idx, row in enumerate(x_des_num):
            label = labels[idx] if idx < len(labels) else f"row_{idx}"
            file.write(f"{label}: ")
            np.savetxt(file, row.reshape(1, -1), fmt="% .4f", delimiter='  ', newline="")
            file.write("\n")

        #file.write("\nswing_inverted:\n")
        for idx, row in enumerate(swing_inverted):
            foot_label = ["FL", "FR", "HL", "HR"][idx]  # Etichette dei piedi
            #file.write(f"{foot_label}: ")
            #np.savetxt(file, row.reshape(1, -1), fmt="% .0f", delimiter='  ', newline="")
            #file.write("\n")
            
        file.write("\nforces:\n")
        for foot, force in forces.items():
            file.write(f"{foot}: {force.tolist()}\n")  # converto numpy array in lista leggibile

        
        fz_total = mpc_class.u[2] + mpc_class.u[5] + mpc_class.u[8] + mpc_class.u[11]

        file.write(f"\nExpected total vertical force: {mpc_class.m * mpc_class.params['g']}")
        file.write(f"\nTotal fz applied: {fz_total:.2f}")
        file.write("\n" + "-"*50 + "\n")


# solves a constrained QP with casadi
class QPSolver:
    def __init__(self, n_vars, n_eq_constraints=0, n_ineq_constraints=0):
        self.n_vars = n_vars
        self.n_eq_constraints = n_eq_constraints
        self.n_ineq_constraints = n_ineq_constraints

        self.opti = ca.Opti('conic')
        self.x = self.opti.variable(self.n_vars)

        # objective function: (1/2) * x.T @ H @ x + F.T @ x
        self.F_ = self.opti.parameter(self.n_vars)
        self.H_ = self.opti.parameter(self.n_vars, self.n_vars)
        objective = 0.5 * self.x.T @ self.H_ @ self.x + self.F_.T @ self.x
        self.opti.minimize(objective)

        # equality constraints: A_eq * x == b_eq
        self.A_eq_ = self.opti.parameter(self.n_eq_constraints, self.n_vars)
        self.b_eq_ = self.opti.parameter(self.n_eq_constraints)
        if self.n_eq_constraints > 0:
            self.opti.subject_to(self.A_eq_ @ self.x == self.b_eq_)

        # inequality constraints: A_ineq * x <= b_ineq
        if self.n_ineq_constraints > 0:
            self.A_ineq_ = self.opti.parameter(self.n_ineq_constraints, self.n_vars)
            self.b_ineq_ = self.opti.parameter(self.n_ineq_constraints)
            self.opti.subject_to(self.A_ineq_ @ self.x <= self.b_ineq_)
        else:
            self.A_ineq_ = None
            self.b_ineq_ = None

        # solver options
        p_opts = {'expand': True}
        s_opts = {'max_iter': 1000, 'verbose': False}
        self.opti.solver('osqp', p_opts, s_opts)

    def set_values(self, H, F, A_eq=None, b_eq=None, A_ineq=None, b_ineq=None):
        self.opti.set_value(self.H_, H)
        self.opti.set_value(self.F_, F)
        if self.n_eq_constraints > 0 and A_eq is not None and b_eq is not None:
            self.opti.set_value(self.A_eq_, A_eq)
            self.opti.set_value(self.b_eq_, b_eq)
        if self.n_ineq_constraints > 0 and A_ineq is not None and b_ineq is not None:
            self.opti.set_value(self.A_ineq_, A_ineq)
            self.opti.set_value(self.b_ineq_, b_ineq)

    def solve(self):
        try:
            solution = self.opti.solve()
            x_sol = solution.value(self.x)
        except RuntimeError as e:
            print("QP Solver failed:", e)
            x_sol = np.zeros(self.n_vars)
        return x_sol
    









#direi di togliere quelli che non usiamo di scianca









def compute_skew(vector):
  v1 = vector[0]
  v2 = vector[1]
  v3 = vector[2]

  #matrix = DM.zeros(3, 3)
  matrix = np.zeros((3,3))
  matrix[0, 1] = -v3
  matrix[1, 0] = v3
  matrix[0, 2] = v2
  matrix[2, 0] = -v2
  matrix[1, 2] = -v1
  matrix[2, 1] = v1 

  return matrix


def display_marker(object, body_name, position_in_world_coords,
                    color, print_bodieds_of_the_object=False):
    # Print the bodies of the object if needed
    if print_bodieds_of_the_object:
        for i in range(object.getNumBodyNodes()):
            print(object.getBodyNode(i).getName())

    # get the body named 'body_name'
    body_node = object.getBodyNode(body_name)

    # display the marker on the body as a ball
    if body_node is None:
        print("Error: BodyNode not found.")
    else:
        sphere = dart.dynamics.SphereShape(0.01)
        sphere_node = body_node.createShapeNode(sphere)

        if sphere_node is None:
            print("Error: Failed to create shape node for the sphere.")
        else:
            vis = sphere_node.createVisualAspect()

            if vis is None:
                print("Error: Failed to get visual aspect.")
            else:
                vis.setColor(color)
                sphere_node.setRelativeTranslation(position_in_world_coords)


def plot_com_and_forces(N, com_position, com_desired, forces, t):
    """
    Plots center of mass positions (actual vs desired) and forces over time.
    
    Parameters:
        N
        com_position: 3xN (x, y, z).
        com_desired: 3xN (x, y, z).
        forces: 4xN.
    """

    time = np.linspace(t, t+ N, N)

    fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    # Plot CoM actual vs desired
    labels = ['x', 'y', 'z']
    for i in range(3):
        axs[0].plot(time, com_position[i, :], label=f'Actual {labels[i]}')
        axs[0].plot(time, com_desired[i, :], '--', label=f'Desired {labels[i]}')

    #label_curr = 2
    #axs[0].plot(time, com_position[label_curr, :], label=f'Actual {labels[label_curr]}')
    #axs[0].plot(time, com_desired[label_curr, :], '--', label=f'Desired {labels[label_curr]}')
    
    axs[0].set_ylabel("CoM Position (m)")
    axs[0].set_title("Center of Mass (CoM) Position Over Time")
    axs[0].legend()
    axs[0].grid(True)

    # Plot Forces
    for i in range(forces.shape[0]):
        axs[1].plot(time, forces[i, :], label=f'Force z {i+1}')
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Force (N)")
    axs[1].set_title("Forces Over Time")
    axs[1].legend()
    axs[1].grid(True)

    plt.tight_layout()
    plt.show()