import numpy as np
from utils import *
import matplotlib.pyplot as plt

class FootstepPlanner:
    def __init__(self, vref, lb_foot, lf_foot, rb_foot, rf_foot, initial_theta, params):
        default_ss_duration = params['ss_duration']
        default_ds_duration = params['ds_duration']

        unicycle_pos   = (lb_foot + rb_foot) / 2.
        print(f"intial pos {unicycle_pos}")
        unicycle_theta = initial_theta

        # lb, rb, lf, rf
        support_foot = [0., 1., 1., 0.] if params['first_swing'] == 'rfoot' else [1., 0., 0., 1.]
        print(f"Initial support foot: {support_foot}")

        self.plan = []

        for j in range(10):
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
                    
                    #unicycle_pos[:2] += R @ vref[:2] * params['world_time_step'] # not use R for numerical approximation error
                    unicycle_pos[:2] += vref[:2] * params['world_time_step']
                    
                    
            
            # compute step position
            displacement_back = 0.0 if support_foot == 'lfoot' else - 0.0
            displ_x_back = - np.sin(unicycle_theta) * displacement_back
            displ_y_back =   np.cos(unicycle_theta) * displacement_back

            torso_displacement = 0.6
            displacement_front = -0.0 if support_foot == 'lfoot' else 0.0
            displ_x_front = - np.sin(unicycle_theta) * displacement_front
            displ_y_front =   np.cos(unicycle_theta) * displacement_front
            
            leg_displacement = (lb_foot[1] + rb_foot[1]) / 2.
            leg_displacement = leg_displacement[0]

            print("----------------------------")
            print(unicycle_pos[0, 0])
            print(displacement_back)
            print(support_foot)
            print("----------------------------")

            pos = {
                    "lb": [
                        unicycle_pos[0, 0] + displacement_back * support_foot[0],  
                        unicycle_pos[1, 0] - leg_displacement
                    ],
                    
                    "rb": [
                        unicycle_pos[0, 0] + displacement_back * support_foot[1], 
                        unicycle_pos[1, 0] + leg_displacement
                    ],
                    
                    "lf": [
                        unicycle_pos[0, 0] + displacement_back * support_foot[2] + torso_displacement, 
                        unicycle_pos[1, 0] - leg_displacement
                    ],

                    "rf": [
                        unicycle_pos[0, 0]  + displacement_back * support_foot[3] + torso_displacement, 
                        unicycle_pos[1, 0]  + leg_displacement
                    ],

                    "ang": unicycle_theta  # Valore Z separato per chiarezza
                }
            
            ang = np.array((0., 0., unicycle_theta))

            # add step to plan
            self.plan.append({
                'pos'        : pos,
                'ang'        : ang,
                'ss_duration': ss_duration,
                'ds_duration': ds_duration,
                'foot_id'    : support_foot
                })
            
            # switch support foot
            support_foot = [1., 0., 0., 1.] if support_foot == [0., 1., 1., 0.]  else [0., 1., 1., 0.]

        x_lb_foot = [step['pos']["lb"][0] for step in self.plan ]
        y_lb_foot = [step['pos']["lb"][1] for step in self.plan ]

        x_rb_foot = [step['pos']["rb"][0] for step in self.plan ]
        y_rb_foot = [step['pos']["rb"][1] for step in self.plan ]

        x_lf_foot = [step['pos']["lf"][0] for step in self.plan ]
        y_lf_foot = [step['pos']["lf"][1] for step in self.plan ]

        x_rf_foot = [step['pos']["rf"][0] for step in self.plan ]
        y_rf_foot = [step['pos']["rf"][1] for step in self.plan ]

        print("Coordinate X ( commented plt.show() per comodita )")
        print("\tDEBUG: sx-dx side non dovrebbero essere uguali")
        print("\tDEBUG: i primi due step sono uguali, perchè?")
        print("\tTEST: usare vref più piccola di 0.5 e.g. 0.1, 0.5 è per debug piu semplice")
        print("x_lb_foot:", [round(x, 2) for x in x_lb_foot])
        print("x_rb_foot:", [round(x, 2) for x in x_rb_foot])
        print("x_lf_foot:", [round(x, 2) for x in x_lf_foot])
        print("x_rf_foot:", [round(x, 2) for x in x_rf_foot])



        plt.figure(figsize=(8, 6))  

        for i in range(len(x_lb_foot)):
            plt.plot(x_lb_foot[i], y_lb_foot[i], 'bo', label='LB Foot' if i == 0 else "")
            plt.text(x_lb_foot[i], y_lb_foot[i], str(i), fontsize=9, verticalalignment='bottom', horizontalalignment='right')

        for i in range(len(x_rb_foot)):
            plt.plot(x_rb_foot[i], y_rb_foot[i], 'ro', label='RB Foot' if i == 0 else "")
            plt.text(x_rb_foot[i], y_rb_foot[i], str(i), fontsize=9, verticalalignment='bottom', horizontalalignment='right')

        for i in range(len(x_lf_foot)):
            plt.plot(x_lf_foot[i], y_lf_foot[i], 'go', label='LF Foot' if i == 0 else "")
            plt.text(x_lf_foot[i], y_lf_foot[i], str(i), fontsize=9, verticalalignment='bottom', horizontalalignment='right')

        for i in range(len(x_rf_foot)):
            plt.plot(x_rf_foot[i], y_rf_foot[i], 'mo', label='RF Foot' if i == 0 else "")
            plt.text(x_rf_foot[i], y_rf_foot[i], str(i), fontsize=9, verticalalignment='bottom', horizontalalignment='right')

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

params = {
        'g': 9.81,
        'h': 0.72,
        'foot_size': 0.1,
        'step_height': 0.02,
        'ss_duration': 70,
        'ds_duration': 30,
        'world_time_step': 0.01,
        'first_swing': 'lfoot',
        'µ': 0.5,
        'N': 100,
        'dof': 30,
    }
    
import numpy as np

lb_foot = np.array([0.0, 0.0, 0.0])
lb_foot.shape = (3,1)  

rb_foot = np.array([0.0, 0.256, 0.0])
rb_foot.shape = (3,1)

lf_foot = np.array([0.6, 0.0, 0.0])
lf_foot.shape = (3,1)

rf_foot = np.array([0.6, 0.256, 0.0])
rf_foot.shape = (3,1)

v_com = np.array([0.5, 0.0, 0.0])
v_com.shape = (3,1)

initial_theta = 0

reference = [(0.1, 0., 0.2)] * 5 + [(0.1, 0., -0.1)] * 10 + [(0.1, 0., 0.)] * 10


footstep_planner = FootstepPlanner(
    v_com,
    lb_foot,
    lf_foot,
    rb_foot,
    rf_foot,
    initial_theta,
    params
    )