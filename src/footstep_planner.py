import numpy as np
from utils import *
import matplotlib.pyplot as plt

class FootstepPlanner:
    """
    The FootstepPlanner is responsible for generating a discrete sequence of foot placements for the Lite3 quadruped robot. 
    It simulates a simplified locomotion model based on a virtual unicycle moving under the robot's center of mass.


    Feet convention: FL = Front Left, FR = Front Right, HL = Hind Left, HR = Hind Right

    Gait encoding:
    - 1 -> foot is in contact with the ground (stance phase)
    - 0 -> foot is swinging (swing phase)

    TO DO:
    - implement the gallopping gait 
    
    Main methods:
    - get_step_index_at_time(time): Returns the step index active at a given time.
    - get_start_time(step_index): Returns the start time of a given step.
    - get_phase_at_time(time): Returns the current foot contact phase at a given time.
    - is_swing(leg_name, gait): Checks whether a given leg is in swing phase based on the gait.
    """



    def __init__(self, initial_configuration, params, show = True):
        ss_duration = params['ss_duration']                
        ds_duration = params['ds_duration']
        vref = params['v_com_ref']
        omegaref = params['theta_dot']
        total_steps = params['total_steps'] 
        support_foot = params['first_swing']  

        fl_foot = initial_configuration['FL_FOOT']
        fr_foot = initial_configuration['FR_FOOT'] 
        hl_foot = initial_configuration['HL_FOOT'] 
        hr_foot = initial_configuration['HR_FOOT']
        initial_theta = initial_configuration['yaw']

        # Let the unicycle start under the CoM
        unicycle_pos   = (hl_foot + hr_foot + fl_foot + fr_foot) / 4.
        unicycle_theta = initial_theta

        # FL, FR, HL, HR  ----------------- FEET ORDER
        self.plan = []

        R = np.array([[np.cos(initial_theta), - np.sin(initial_theta)],
                      [np.sin(initial_theta),   np.cos(initial_theta)]])

        if total_steps == 0:
            for j in range(100):    
                # set step duration
                pos = {
                    'FL_FOOT' : fl_foot,
                    'FR_FOOT' : fr_foot,
                    'HL_FOOT' : hl_foot,
                    'HR_FOOT' : hr_foot
                }
                ang = unicycle_theta 
                self.plan.append({
                        'pos'        : pos,
                        'ang'        : ang,
                        'ss_duration': ss_duration,
                        'ds_duration': ds_duration,
                        'feet_id'    : [1,1,1,1]
                        })
            return 
                          
        for j in range(total_steps):

            # exception for last step TODO 
            # to be added

            # move virtual unicycle 
            for i in range(ss_duration + ds_duration):
                if j >= 1:
                    unicycle_theta += omegaref * params['world_time_step']
                    R = np.array([[np.cos(unicycle_theta), - np.sin(unicycle_theta)],
                                  [np.sin(unicycle_theta),   np.cos(unicycle_theta)]])
                    
                    unicycle_pos[:2] += R @ vref[:2] * params['world_time_step']

            # compute step position
            torso_displacement = R @ (fl_foot[:2]-hl_foot[:2])
            leg_displacement_y = R @ (hr_foot[:2]- hl_foot[:2])/2.
 

            if j >= 1 and j < total_steps: # Lite3 walk

                pos = {
                        "FL_FOOT": [
                            self.plan[j-1]['pos']['FL_FOOT'][0] if support_foot[0] else unicycle_pos[0] + torso_displacement[0]/2  - leg_displacement_y[0], #tolto + leg_displacement_x[0]
                            self.plan[j-1]['pos']['FL_FOOT'][1] if support_foot[0] else unicycle_pos[1] + torso_displacement[1]/2  - leg_displacement_y[1],
                            self.plan[j-1]['pos']['FL_FOOT'][2] if support_foot[0] else unicycle_pos[2]
                        ],      

                        "FR_FOOT": [
                            self.plan[j-1]['pos']['FR_FOOT'][0] if support_foot[1] else unicycle_pos[0]  + torso_displacement[0]/2  + leg_displacement_y[0],
                            self.plan[j-1]['pos']['FR_FOOT'][1] if support_foot[1] else unicycle_pos[1]  + torso_displacement[1]/2  + leg_displacement_y[1],
                            self.plan[j-1]['pos']['FR_FOOT'][2] if support_foot[1] else unicycle_pos[2]
                        ],

                        "HL_FOOT": [
                            self.plan[j-1]['pos']['HL_FOOT'][0] if support_foot[2] else unicycle_pos[0]  - torso_displacement[0]/2 - leg_displacement_y[0],
                            self.plan[j-1]['pos']['HL_FOOT'][1] if support_foot[2] else unicycle_pos[1]  - torso_displacement[1]/2 - leg_displacement_y[1],
                            self.plan[j-1]['pos']['HL_FOOT'][2] if support_foot[2] else unicycle_pos[2]
                        ],
                        
                        "HR_FOOT": [
                            self.plan[j-1]['pos']['HR_FOOT'][0] if support_foot[3] else unicycle_pos[0] - torso_displacement[0]/2 + leg_displacement_y[0], 
                            self.plan[j-1]['pos']['HR_FOOT'][1] if support_foot[3] else unicycle_pos[1] - torso_displacement[1]/2 + leg_displacement_y[1],
                            self.plan[j-1]['pos']['HR_FOOT'][2] if support_foot[3] else unicycle_pos[2]
                        ],
                        "hip":[
                            unicycle_pos[0] - torso_displacement[0]/2, 
                            unicycle_pos[1] - torso_displacement[1]/2,
                            params['h']  
                        ]
                    }
                
            else: # Lite3 Stand-up
                pos = {
                        "FL_FOOT": [
                            unicycle_pos[0] + torso_displacement[0]/2 - leg_displacement_y[0], 
                            unicycle_pos[1] + torso_displacement[1]/2 - leg_displacement_y[1],
                            unicycle_pos[2]
                        ],

                        "FR_FOOT": [
                            unicycle_pos[0]  + torso_displacement[0]/2 + leg_displacement_y[0], 
                            unicycle_pos[1]  + torso_displacement[1]/2 + leg_displacement_y[1],
                            unicycle_pos[2]
                        ], 

                        "HL_FOOT": [
                            unicycle_pos[0] - torso_displacement[0]/2 - leg_displacement_y[0],
                            unicycle_pos[1] - torso_displacement[1]/2 - leg_displacement_y[1],
                            unicycle_pos[2]
                        ],
                        
                        "HR_FOOT": [
                            unicycle_pos[0] - torso_displacement[0]/2 + leg_displacement_y[0], 
                            unicycle_pos[1] - torso_displacement[1]/2 + leg_displacement_y[1],
                            unicycle_pos[2]
                        ],
                        "hip":[
                            unicycle_pos[0] - torso_displacement[0]/2, 
                            unicycle_pos[1] - torso_displacement[1]/2,
                            params['h']  
                        ]
                    }
            
            ang = np.array((0., 0., unicycle_theta))

            # add step to plan
            if j == 0:
                self.plan.append({
                    'pos'        : pos,
                    'ang'        : ang,
                    'ss_duration': ss_duration,
                    'ds_duration': ds_duration,
                    'feet_id'    : [1,1,1,1]
                    })
            else:
                self.plan.append({
                    'pos'        : pos,
                    'ang'        : ang,
                    'ss_duration': ss_duration,
                    'ds_duration': ds_duration,
                    'feet_id'    : support_foot
                    })
            # switch support foot
            if j > 0 and j < total_steps:
                support_foot = np.array([1,1,1,1]) - support_foot
    

        if show == True:
            x_hl_foot = [step['pos']["HL_FOOT"][0] for step in self.plan ]
            y_hl_foot = [step['pos']["HL_FOOT"][1] for step in self.plan ]

            x_hr_foot = [step['pos']["HR_FOOT"][0] for step in self.plan ]
            y_hr_foot = [step['pos']["HR_FOOT"][1] for step in self.plan ]

            x_fl_foot = [step['pos']["FL_FOOT"][0] for step in self.plan ]
            y_fl_foot = [step['pos']["FL_FOOT"][1] for step in self.plan ]

            x_fr_foot = [step['pos']["FR_FOOT"][0] for step in self.plan ]
            y_fr_foot = [step['pos']["FR_FOOT"][1] for step in self.plan ]

            x_hip = [step['pos']["hip"][0] for step in self.plan ]
            y_hip = [step['pos']["hip"][1] for step in self.plan ]
            
            
            
            plt.figure(figsize=(8, 6))  
        
            for i in range(len(x_hip)):
                v_al = 'bottom' if i%2==0 else 'top'
                plt.plot(x_hip[i], y_hip[i], 'wo', label='hip' if i == 0 else "")
                plt.text(x_hip[i], y_hip[i], str(i), fontsize=12, verticalalignment=v_al, horizontalalignment='right')
    
                plt.plot(x_hr_foot[i], y_hr_foot[i], 'ro', label='RB Foot' if i == 0 else "")
                plt.text(x_hr_foot[i], y_hr_foot[i], str(i), fontsize=12, verticalalignment=v_al, horizontalalignment='right')

                plt.plot(x_hl_foot[i], y_hl_foot[i], 'bo', label='LB Foot' if i == 0 else "")
                plt.text(x_hl_foot[i], y_hl_foot[i], str(i), fontsize=12, verticalalignment=v_al, horizontalalignment='right')

                plt.plot(x_fl_foot[i], y_fl_foot[i], 'go', label='LF Foot' if i == 0 else "")
                plt.text(x_fl_foot[i], y_fl_foot[i], str(i), fontsize=9, verticalalignment=v_al, horizontalalignment='right')

                plt.plot(x_fr_foot[i], y_fr_foot[i], 'mo', label='RF Foot' if i == 0 else "")
                plt.text(x_fr_foot[i], y_fr_foot[i], str(i), fontsize=9, verticalalignment=v_al, horizontalalignment='right')
            
            plt.xlabel("X Position (m)")
            plt.ylabel("Y Position (m)")
            plt.title("Footstep Plan (LB, RB, LF, RF)")
            plt.legend()
            plt.grid(True)
            
            plt.show()
        
        
    def get_step_index_at_time(self, time):
        t = 0
        for i in range(len(self.plan)):
            t += self.plan[i]['ss_duration'] + self.plan[i]['ds_duration']
            if t > time: return i
        return len(self.plan)-1

    def get_start_time(self, step_index):
        t = 0
        for i in range(step_index):
            t += self.plan[i]['ss_duration'] + self.plan[i]['ds_duration']
        return t

    def get_phase_at_time(self, time):
        step_index = self.get_step_index_at_time(time)
        start_time = self.get_start_time(step_index)
        time_in_step = time - start_time
        if time_in_step < self.plan[step_index]['ss_duration']:
            return self.plan[step_index]['feet_id']
        else:
            return [1, 1, 1, 1]

    def is_swing(self, leg_name, gait):
        if leg_name == 'FL_FOOT':
            return 1-gait[0]
        if leg_name == 'FR_FOOT':
            return 1-gait[1]
        if leg_name == 'HL_FOOT':
            return 1-gait[2]
        if leg_name == 'HR_FOOT':
            return 1-gait[3]