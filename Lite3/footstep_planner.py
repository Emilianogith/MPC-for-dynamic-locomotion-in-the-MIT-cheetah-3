import numpy as np
from utils import *
import matplotlib.pyplot as plt

class FootstepPlanner:
    def __init__(self, vref, fl_foot, fr_foot, hl_foot, hr_foot, initial_theta, params):
        default_ss_duration = params['ss_duration']
        default_ds_duration = params['ds_duration']

        unicycle_pos   = (hl_foot + hr_foot) / 2.
        unicycle_theta = initial_theta

        # FL, FR, HL, HR
        support_foot = params['first_swing']
        self.plan = []

        total_steps = 10
        for j in range(total_steps):
            # set step duration
            ss_duration = default_ss_duration
            ds_duration = default_ds_duration

            # exception for first step
            if j == 0:
                ss_duration = 0  
                ds_duration = (default_ss_duration + default_ds_duration) * 2

            # exception for last step
            # to be added

            # move virtual unicycle
            for i in range(ss_duration + ds_duration):
                if j > 1:
                    unicycle_theta += vref[2][0] * params['world_time_step']
                    R = np.array([[np.cos(unicycle_theta), - np.sin(unicycle_theta)],
                                  [np.sin(unicycle_theta),   np.cos(unicycle_theta)]])
                    
                    unicycle_pos[:2] += R @ vref[:2] * params['world_time_step'] # not use R for numerical approximation error
                    #unicycle_pos[:2] += vref[:2] * params['world_time_step']
            print(unicycle_pos[:2])
                    
            # compute step position
            torso_displacement = (fl_foot[0]-hl_foot[0])
            torso_displacement = torso_displacement[0] 
            leg_displacement_y = (hr_foot[1]- hl_foot[1])/2.
            leg_displacement_y = leg_displacement_y[0] 
            leg_displacement_x  = 0.001  # length of the step

            # if j == total_steps-1: # Lite3 end its walk ( no more steps ahead )
            #     pos = {
            #             "FL": [
            #                 unicycle_pos[0, 0] + torso_displacement + leg_displacement_x, 
            #                 unicycle_pos[1, 0] - leg_displacement_y
            #             ],

            #             "FR": [
            #                 unicycle_pos[0, 0]  + torso_displacement + leg_displacement_x,
            #                 unicycle_pos[1, 0]  + leg_displacement_y, 
            #             ],

            #             "HL": [
            #                 unicycle_pos[0, 0] + leg_displacement_x,
            #                 unicycle_pos[1, 0] - leg_displacement_y 
            #             ],
                        
            #             "HR": [
            #                 unicycle_pos[0, 0] + leg_displacement_x, 
            #                 unicycle_pos[1, 0] + leg_displacement_y
            #             ],
                        
            #             "hip":[
            #                 unicycle_pos[0, 0], 
            #                 unicycle_pos[1, 0],  
            #             ],

            #             "ang": unicycle_theta  
            #         }
            if j > 0: # Lite3 walk
                pos = {
                        "FL": [
                            self.plan[j-1]['pos']['FL'][0] if support_foot[2] else unicycle_pos[0, 0] + torso_displacement + leg_displacement_x, 
                            self.plan[j-1]['pos']['FL'][1] if support_foot[2] else unicycle_pos[1, 0] - leg_displacement_y
                        ],      

                        "FR": [
                            self.plan[j-1]['pos']['FR'][0] if support_foot[3] else unicycle_pos[0, 0]  + torso_displacement + leg_displacement_x,
                            self.plan[j-1]['pos']['FR'][1] if support_foot[3] else unicycle_pos[1, 0]  + leg_displacement_y, 
                        ],

                        "HL": [
                            self.plan[j-1]['pos']['HL'][0] if support_foot[0] else unicycle_pos[0, 0] + leg_displacement_x,
                            self.plan[j-1]['pos']['HL'][1] if support_foot[0] else unicycle_pos[1, 0] - leg_displacement_y 
                        ],
                        
                        "HR": [
                            self.plan[j-1]['pos']['HR'][0] if support_foot[1] else unicycle_pos[0, 0] + leg_displacement_x, 
                            self.plan[j-1]['pos']['HR'][1] if support_foot[1] else unicycle_pos[1, 0] + leg_displacement_y
                        ],
                         
                        "hip":[
                            unicycle_pos[0, 0], 
                            unicycle_pos[1, 0],  
                        ],

                        "ang": unicycle_theta  
                    }
            else: # Lite3 Stand-up
                pos = {
                        "FL": [
                            unicycle_pos[0, 0] + torso_displacement, 
                            unicycle_pos[1, 0] - leg_displacement_y
                        ],

                        "FR": [
                            unicycle_pos[0, 0]  + torso_displacement, 
                            unicycle_pos[1, 0]  + leg_displacement_y
                        ], 

                        "HL": [
                            unicycle_pos[0, 0],
                            unicycle_pos[1, 0] - leg_displacement_y 
                        ],
                        
                        "HR": [
                            unicycle_pos[0, 0], 
                            unicycle_pos[1, 0] + leg_displacement_y
                        ],
                        
                        "hip":[
                            unicycle_pos[0, 0], 
                            unicycle_pos[1, 0],  
                        ],

                        "ang": unicycle_theta  
                    }
            
            ang = np.array((0., 0., unicycle_theta))

            # add step to plan
            self.plan.append({
                'pos'        : pos,
                'ang'        : ang,
                'ss_duration': ss_duration,
                'ds_duration': ds_duration,
                'feet_id'    : support_foot
                })
            
            # switch support foot
            if j > 0:
                support_foot = np.array([1,1,1,1]) - support_foot

        x_hl_foot = [step['pos']["HL"][0] for step in self.plan ]
        y_hl_foot = [step['pos']["HL"][1] for step in self.plan ]

        x_hr_foot = [step['pos']["HR"][0] for step in self.plan ]
        y_hr_foot = [step['pos']["HR"][1] for step in self.plan ]

        x_fl_foot = [step['pos']["FL"][0] for step in self.plan ]
        y_fl_foot = [step['pos']["FL"][1] for step in self.plan ]

        x_fr_foot = [step['pos']["FR"][0] for step in self.plan ]
        y_fr_foot = [step['pos']["FR"][1] for step in self.plan ]

        x_hip = [step['pos']["hip"][0] for step in self.plan ]
        y_hip = [step['pos']["hip"][1] for step in self.plan ]

        #print("x_hl_foot:\t", [round(x, 2) for x in x_hl_foot])
        #print("x_hr_foot:\t", [round(x, 2) for x in x_hr_foot])
        #print("x_fl_foot:\t", [round(x, 2) for x in x_fl_foot])
        #print("x_fr_foot:\t", [round(x, 2) for x in x_fr_foot])
        #print("x_hip:\t\t", [round(x, 2) for x in x_hip])


        plt.figure(figsize=(8, 6))  

        for i in range(len(x_hip)):
            v_al = 'bottom' if i%2==0 else 'top'
            plt.plot(x_hip[i], y_hip[i], 'go', label='hip' if i == 0 else "")
            plt.text(x_hip[i], y_hip[i], str(i), fontsize=12, verticalalignment=v_al, horizontalalignment='right')
 
            plt.plot(x_hr_foot[i], y_hr_foot[i], 'ro', label='RB Foot' if i == 0 else "")
            plt.text(x_hr_foot[i], y_hr_foot[i], str(i), fontsize=12, verticalalignment=v_al, horizontalalignment='right')

            plt.plot(x_hl_foot[i], y_hl_foot[i], 'bo', label='LB Foot' if i == 0 else "")
            plt.text(x_hl_foot[i], y_hl_foot[i], str(i), fontsize=12, verticalalignment=v_al, horizontalalignment='right')

            #plt.plot(x_fl_foot[i], y_fl_foot[i], 'go', label='LF Foot' if i == 0 else "")
            #plt.text(x_fl_foot[i], y_fl_foot[i], str(i), fontsize=9, verticalalignment=v_al, horizontalalignment='right')

            #plt.plot(x_fr_foot[i], y_fr_foot[i], 'mo', label='RF Foot' if i == 0 else "")
            #plt.text(x_fr_foot[i], y_fr_foot[i], str(i), fontsize=9, verticalalignment=v_al, horizontalalignment='right')
        
        # Impostazioni del grafico
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("Footstep Plan (LB, RB, LF, RF)")
        plt.legend()
        plt.grid(True)

        #plt.show()

    def get_step_index_at_time(self, time):
        t = 0
        for i in range(len(self.plan)):
            t += self.plan[i]['ss_duration'] + self.plan[i]['ds_duration']
            if t > time: return i
        return None

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
            return 'ss'
        else:
            return 'ds'
