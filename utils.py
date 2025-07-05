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


def compute_skew(vector):
  v1 = vector[0]
  v2 = vector[1]
  v3 = vector[2]

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
    Plots center of mass positions (predicted vs reference) and forces over time.
    
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
        axs[0].plot(time, com_position[i, :], label=f'Predicted {labels[i]}')
        axs[0].plot(time, com_desired[i, :], '--', label=f'Reference {labels[i]}')

    #label_curr = 2
    #axs[0].plot(time, com_position[label_curr, :], label=f'Actual {labels[label_curr]}')
    #axs[0].plot(time, com_desired[label_curr, :], '--', label=f'Desired {labels[label_curr]}')
    
    axs[0].set_ylabel("CoM Position (m)")
    axs[0].set_title(f"Center of Mass (CoM) Prediction Over Time step at time step {t}")
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


def plot_trajectory(total_sim_steps, com_position, com_desired, time_step, title = 'Position'):
    """
    Plots center of mass trtajecrtory during the simulation (actual vs desired) over time.
    
    Parameters:
        total_time
        com_position: 3xN (x, y, z).
        com_desired: 3xN (x, y, z).
    """
    total_time = total_sim_steps * time_step
    time = np.arange(0, total_time, time_step)

    fig, axs = plt.subplots(figsize=(12, 8))
    
    # Plot CoM actual vs desired
    if title == 'Orientation':
        labels = ['roll', 'pitch', 'yaw']
    else:
        labels = ['x', 'y', 'z']
    for i in range(3):
        axs.plot(time, com_position[i, :], label=f'Actual {labels[i]}')
        axs.plot(time, com_desired[i, :], '--', label=f'Desired {labels[i]}')

    
    axs.set_ylabel(f"CoM {title} (m)")
    axs.set_xlabel("Time (s)")
    axs.set_title(f"Center of Mass (CoM) {title} Over Time")
    axs.legend()
    axs.grid(True)

    plt.show()



def plot_feet(total_sim_steps, feet, feet_des, time_step, foot_name):
    """
    Plots center of mass trtajecrtory during the simulation (actual vs desired) over time.
    
    Parameters:
        total_time
        feet_z: 1xN (z).
        feet_z_des: 1xN (z).
    """
    total_time = total_sim_steps * time_step
    time = np.arange(0, total_time, time_step)

    fig, axs = plt.subplots(figsize=(12, 8))

    axs.plot(time, feet, label=f'Actual {foot_name} z')
    axs.plot(time, feet_des, '--', label=f'Desired {foot_name} z')

    
    axs.set_ylabel("Foot Position Z (m)")
    axs.set_xlabel("Time (s)")
    axs.set_title(f"{foot_name} Position Z Over Time")
    axs.legend()
    axs.grid(True)

    plt.show()



def plot_forces(total_sim_steps, forces, time_step, foot_name, title=None):
    total_time = total_sim_steps * time_step
    time = np.arange(0, total_time, time_step)

    fig, axs = plt.subplots(figsize=(12, 8))

    # Plot CoM actual vs desired
    labels = list(forces.keys())
    for i in range(3):
        axs.plot(time, forces[labels[i]], label=f'Actual {labels[i]}')
    
    axs.set_ylabel("Forces (N)")
    axs.set_xlabel("Time (s)")
    axs.set_title(f"{foot_name} {title} Over Time")
    axs.legend()
    axs.grid(True)
    plt.show()