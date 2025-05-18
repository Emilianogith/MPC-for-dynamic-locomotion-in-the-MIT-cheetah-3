import numpy as np
from utils import *
import matplotlib.pyplot as plt

class FootstepPlanner:
    """Brief explanation of Footstep planner in the init it is planned ...
     spiega i passi 


     TO DO 

     1->piede in movimento
     0->piede fermo non lo so 

    main methods:
    - get_step_index_at_time
    - get_start_time
    - get_phase_at_time
    - is_swing                    .....

    
    """

    def __init__(self, initial_configuration, params, show = False):
        default_ss_duration = params['ss_duration']                
        default_ds_duration = params['ds_duration']
        vref = params['v_com_ref']
        omegaref = params['theta_dot']
        total_steps = params['total_steps'] 
        support_foot = params['first_swing']  

        fl_foot = initial_configuration['FL_FOOT']
        fr_foot = initial_configuration['FR_FOOT'] 
        hl_foot = initial_configuration['HL_FOOT'] 
        hr_foot = initial_configuration['HR_FOOT']
        initial_theta = initial_configuration['yaw']

        # Let the unicycle start nemehat the Hind hip
        unicycle_pos   = (hl_foot + hr_foot) / 2.
        unicycle_theta = initial_theta






        #params['world_time_step']*= 10

        # total_distance = 0.8

        # total_time = (total_distance/vref[0])/params['world_time_step']
        # print(f'trajectory execution time: {total_distance/vref[0]}, totatl time steps: {total_time}')

        # total_steps = int(total_time/(default_ds_duration + default_ss_duration))
        # 
        #ds_duration = int((total_time/total_steps)-default_ss_duration)
        # print(total_steps)



        #vref*total_time=single_step_distance*total_steps
        #single_step_distance = vref*(ss_duration + ds_duration)
        #ds_duration = single_step_distance/vref-ss_duration
        #single_step_time = (ss_duration + ds_duration)*params['world_time_step']
        # velocità e steps vincolati

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
                        'ss_duration': default_ss_duration,
                        'ds_duration': default_ds_duration,
                        'feet_id'    : [1,1,1,1]
                        })
            return 
                          
        for j in range(total_steps):

            ss_duration = default_ss_duration
            ds_duration = default_ds_duration

            ## exception for first step
            #if j == 0:
            #    ss_duration = 0  
            #    ds_duration = (default_ss_duration + default_ds_duration)

            # exception for last step TODO 
            # to be added

            # move virtual unicycle
            half_stance_time = int(ds_duration / 2)
            
            for i in range(half_stance_time+80): #ss_duration):
                if j >= 1:
                    unicycle_theta += omegaref * params['world_time_step']
                    R = np.array([[np.cos(unicycle_theta), - np.sin(unicycle_theta)],
                                  [np.sin(unicycle_theta),   np.cos(unicycle_theta)]])
                    
                    unicycle_pos[:2] += R @ vref[:2] * params['world_time_step']

            # compute step position
            torso_displacement = R @ (fl_foot[:2]-hl_foot[:2])
            leg_displacement_y = R @ (hr_foot[:2]- hl_foot[:2])/2.

            # leg_displacement_x = R @ vref[:2].reshape(2,1)                        # QUESTO LEG DISPLACEMENT X CI FA FARE IL PRIMO PASSO PIU LUNGO E COMUNQUE NOIN SERVE A NIENTE 
            # leg_displacement_x = leg_displacement_x.flatten()
            # leg_displacement_x = [0,0]
 

            if j >= 1 and j < total_steps: # Lite3 walk

                pos = {
                        "FL_FOOT": [
                            self.plan[j-1]['pos']['FL_FOOT'][0] if support_foot[0] else unicycle_pos[0] + torso_displacement[0]  - leg_displacement_y[0], #tolto + leg_displacement_x[0]
                            self.plan[j-1]['pos']['FL_FOOT'][1] if support_foot[0] else unicycle_pos[1] + torso_displacement[1]  - leg_displacement_y[1],
                            self.plan[j-1]['pos']['FL_FOOT'][2] if support_foot[0] else unicycle_pos[2]
                        ],      

                        "FR_FOOT": [
                            self.plan[j-1]['pos']['FR_FOOT'][0] if support_foot[1] else unicycle_pos[0]  + torso_displacement[0]  + leg_displacement_y[0],
                            self.plan[j-1]['pos']['FR_FOOT'][1] if support_foot[1] else unicycle_pos[1]  + torso_displacement[1]  + leg_displacement_y[1],
                            self.plan[j-1]['pos']['FR_FOOT'][2] if support_foot[1] else unicycle_pos[2]
                        ],

                        "HL_FOOT": [
                            self.plan[j-1]['pos']['HL_FOOT'][0] if support_foot[2] else unicycle_pos[0]  - leg_displacement_y[0],
                            self.plan[j-1]['pos']['HL_FOOT'][1] if support_foot[2] else unicycle_pos[1]  - leg_displacement_y[1],
                            self.plan[j-1]['pos']['HL_FOOT'][2] if support_foot[2] else unicycle_pos[2]
                        ],
                        
                        "HR_FOOT": [
                            self.plan[j-1]['pos']['HR_FOOT'][0] if support_foot[3] else unicycle_pos[0] + leg_displacement_y[0], 
                            self.plan[j-1]['pos']['HR_FOOT'][1] if support_foot[3] else unicycle_pos[1] + leg_displacement_y[1],
                            self.plan[j-1]['pos']['HR_FOOT'][2] if support_foot[3] else unicycle_pos[2]
                        ],
                        "hip":[
                            unicycle_pos[0], 
                            unicycle_pos[1],
                            params['h']  #check
                        ]
                    }
                
            else: # Lite3 Stand-up
                pos = {
                        "FL_FOOT": [
                            unicycle_pos[0] + torso_displacement[0] - leg_displacement_y[0], 
                            unicycle_pos[1] + torso_displacement[1] - leg_displacement_y[1],
                            unicycle_pos[2]
                        ],

                        "FR_FOOT": [
                            unicycle_pos[0]  + torso_displacement[0] + leg_displacement_y[0], 
                            unicycle_pos[1]  + torso_displacement[1] + leg_displacement_y[1],
                            unicycle_pos[2]
                        ], 

                        "HL_FOOT": [
                            unicycle_pos[0] - leg_displacement_y[0],
                            unicycle_pos[1] - leg_displacement_y[1],
                            unicycle_pos[2]
                        ],
                        
                        "HR_FOOT": [
                            unicycle_pos[0] + leg_displacement_y[0], 
                            unicycle_pos[1] + leg_displacement_y[1],
                            unicycle_pos[2]
                        ],
                        "hip":[
                            unicycle_pos[0], 
                            unicycle_pos[1],
                            params['h']  #check
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
            #elif j == total_steps:                                     # se vogliamo fargli trackare la traiettoria con v costrante noin ha senso farlo stare fermo all'inizio
            #    support_foot = np.array([1,1,1,1])


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
            
            # Impostazioni del grafico
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