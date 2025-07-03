from utils import *
from logger import Logger


PLOT_MPC = False
PLOT_TRAJ = True
PLOT_FEET_Z = False
PLOT_FORCES = True
PLOT_FEET_X = False


plot_keys = {'params' : None,
            "total_sim_steps" : 0}
logger  = Logger(plot_keys)

# load logger data
logger.load_log()

# print simulation info 
logger.print_log_info()

total_sim_steps = logger.log["total_sim_steps"]
time_step = logger.log["sim_params"]['world_time_step']

# plot MPC PREDICTIONS
if PLOT_MPC:
    for data in logger.log["MPC PREDICTIONS"]:
        t = data['time step']
        N = data['predicted_state'].shape[1]-1
        com_position = data['predicted_state'][3:6,:N]
        com_desired = data['desired_state'][3:6,:N]
        forces = data[ 'predicted forces']

        plot_com_and_forces(N, com_position, com_desired, forces, t)

# plot tracking performance
if PLOT_TRAJ:
    actual_state_trajectory = logger.log["TRACKING PERFORMANCE"]['actual']
    desired_state_trajectory = logger.log["TRACKING PERFORMANCE"]['desired']
    com_position = np.array([elem[3:6] for elem in actual_state_trajectory]).T
    com_desired = np.array([elem[3:6] for elem in desired_state_trajectory]).T

    plot_trajectory(total_sim_steps, com_position, com_desired, time_step)

    com_orientation = np.array([elem[:3] for elem in actual_state_trajectory]).T
    com_orientation_desired = np.array([elem[:3] for elem in desired_state_trajectory]).T

    plot_trajectory(total_sim_steps, com_orientation, com_orientation_desired, time_step, title='Orientation')

# plot feet z trajectory
if PLOT_FEET_Z:
    foot_name = 'FL_FOOT'
    foot_traj = logger.log["FEET POS"][foot_name]['actual']
    foot_des_traj = logger.log["FEET POS"][foot_name]['des']
    foot_z = np.array([elem[2] for elem in foot_traj]).T
    foot_z_des = np.array([elem[2] for elem in foot_des_traj]).T

    plot_feet(total_sim_steps, foot_z, foot_z_des, time_step, foot_name)

if PLOT_FEET_X:
    foot_name = 'FL_FOOT'
    foot_traj = logger.log["FEET POS"][foot_name]['actual']
    foot_des_traj = logger.log["FEET POS"][foot_name]['des']
    foot_z = np.array([elem[0] for elem in foot_traj]).T
    foot_z_des = np.array([elem[0] for elem in foot_des_traj]).T

    plot_feet(total_sim_steps, foot_z, foot_z_des, time_step, foot_name)   


if PLOT_FORCES:
    foot_name = 'FL_FOOT'
    forces = logger.log["FORCES"][foot_name]
    plot_forces(total_sim_steps, forces, time_step, foot_name, title = 'Force Magnitude')


    effort = logger.log["CONTROL EFFORT"][foot_name]
    plot_forces(total_sim_steps, effort, time_step, foot_name, title = 'Control Effort')

