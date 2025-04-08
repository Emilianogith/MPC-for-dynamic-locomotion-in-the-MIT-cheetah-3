import numpy as np
import matplotlib.pyplot as plt 

class FootTrajectoryGenerator:
    def __init__(self, footstep_planner, params):
        self.dt = params['world_time_step']
        self.step_height = params['step_height']
        self.footstep_planner = footstep_planner
        self.plan = self.footstep_planner.plan

    def generate_feet_trajectories_at_time(self, time, foot):
        step_index = self.footstep_planner.get_step_index_at_time(time)
        time_in_step = time - self.footstep_planner.get_start_time(step_index)
        #gait = self.footstep_planner.plan[step_index]['feet_id']
        ss_duration = self.footstep_planner.plan[step_index]['ss_duration']
        
        start_pos  = np.array(self.plan[step_index-1]['pos'][foot])
        target_pos = np.array(self.plan[step_index+1]['pos'][foot])
        start_ang  = np.array(self.plan[step_index-1]['ang'])
        target_ang = np.array(self.plan[step_index+1]['ang'])

        if step_index == 0:
            return {
                'pos': np.hstack((start_ang, start_pos)),
                'vel': np.zeros(6),
                'acc': np.zeros(6)
            }

        t = time_in_step
        T = ss_duration

        if t >= T:
            print('totti')
            self.plan[step_index]['feet_id'] = [1,1,1,1]
            return {
                'pos': np.hstack((start_ang, start_pos)),
                'vel': np.zeros(6),
                'acc': np.zeros(6)
            }
        
        # cubic polynomial for position and angle
        A = - 2 / T**3
        B =   3 / T**2
        swing_pos     = start_pos + (target_pos - start_pos) * (    A * t**3 +     B * t**2)
        swing_vel     =             (target_pos - start_pos) * (3 * A * t**2 + 2 * B * t   ) / self.dt
        swing_acc     =             (target_pos - start_pos) * (6 * A * t    + 2 * B       ) / self.dt**2
        swing_ang_pos = start_ang + (target_ang - start_ang) * (    A * t**3 +     B * t**2)
        swing_ang_vel =             (target_ang - start_ang) * (3 * A * t**2 + 2 * B * t   ) / self.dt
        swing_ang_acc =             (target_ang - start_ang) * (6 * A * t    + 2 * B       ) / self.dt**2

        # quartic polynomial for vertical position
        A =   16 * self.step_height / T**4
        B = - 32 * self.step_height / T**3
        C =   16 * self.step_height / T**2
        swing_pos[2] =       A * t**4 +     B * t**3 +     C * t**2
        swing_vel[2] = ( 4 * A * t**3 + 3 * B * t**2 + 2 * C * t   ) / self.dt
        swing_acc[2] = (12 * A * t**2 + 6 * B * t    + 2 * C       ) / self.dt**2

        # assemble pose, velocity, and acceleration for swing foot
        swing_data = {
            'pos': np.hstack((swing_ang_pos, swing_pos)),
            'vel': np.hstack((swing_ang_vel, swing_vel)),
            'acc': np.hstack((swing_ang_acc, swing_acc))
        }
        print('--------------------------------------------------------------')
        print(swing_data['pos'])
        return swing_data
    

    def show_trajectory(self, foot_to_sample, t_start=0, t_end=1000, string_axs='z'):
        time_samples = np.linspace(t_start, t_end, num=t_end-t_start)

        removed_samples = []

        swing_pos_list = []
        swing_vel_list = []
        swing_acc_list = []

        for i, t in enumerate(time_samples):
            if self.footstep_planner.get_step_index_at_time(t) < 0:
                removed_samples.append(i)
                continue

            foot_trajectory = self.generate_feet_trajectories_at_time(t, foot_to_sample)

            swing_pos = foot_trajectory['pos'][3:]
            swing_vel = foot_trajectory['vel'][3:]
            swing_acc = foot_trajectory['acc'][3:]

            swing_pos_list.append(swing_pos)
            swing_vel_list.append(swing_vel)
            swing_acc_list.append(swing_acc)

        swing_pos_array = np.array(swing_pos_list)
        swing_vel_array = np.array(swing_vel_list)
        swing_acc_array = np.array(swing_acc_list)

        time_samples = np.delete(time_samples, removed_samples)
        
        # Plot della traiettoria
        fig, axs = plt.subplots(3, 1, figsize=(10, 8))

        string_axs = string_axs.lower()
        plot_axs = 2 if string_axs=='z' else ( 1 if string_axs=='y' else 0 )

        axs[0].plot(time_samples, swing_pos_array[:, plot_axs])
        axs[0].set_ylabel("Position")
        axs[0].legend()
        axs[0].grid()

        axs[1].plot(time_samples, swing_vel_array[:, plot_axs], color='orange')
        axs[1].set_ylabel("velocity")
        axs[1].legend()
        axs[1].grid()

        axs[2].plot(time_samples, swing_acc_array[:, plot_axs], color='red')
        axs[2].set_xlabel("Time")
        axs[2].set_ylabel("Acceleration")
        axs[2].legend()
        axs[2].grid()

        string_axs = 'x' if plot_axs==0 else 'y'
        if plot_axs==2:
            string_axs = 'z'

        plt.suptitle(f"Trajectory of {foot_to_sample} on {string_axs} axis")
        plt.show()

        return   