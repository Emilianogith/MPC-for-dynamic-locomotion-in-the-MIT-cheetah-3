import numpy as np

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
        
        start_pos  = self.plan[step_index - 1]['pos'][foot]
        target_pos = self.plan[step_index + 1]['pos'][foot]
        start_ang  = self.plan[step_index - 1]['ang'][foot]
        target_ang = self.plan[step_index + 1]['ang'][foot]
    
        t = time_in_step
        T = ss_duration

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
        return swing_data
        