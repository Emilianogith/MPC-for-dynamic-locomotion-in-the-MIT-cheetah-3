import numpy as np
from matplotlib import pyplot as plt
import pickle

class Logger():
    """Log simulation data.
    initial: 
    -mpc freq
    -all simulation params (gait, sim time step, ...)
    -time array
    -plot feet z over time
    -plot horizon MPC in 3 different time steps
    -tracking error of the state (reference vs executed)
    -forces xyz for the feet
    -control effort torques tau
    -total sim time
    
    """


    def __init__(self, initial):
        self.log = {"mpc_freq" : 0,
                    "sim_params": initial['params'], 
                    "total_sim_steps" : initial['total_sim_steps'],          
                    "time array" : [], 
                    "FEET POS" : { 'FL_FOOT' : {'actual' : [],        
                                               'des' : []},
                                 'FR_FOOT' :{'actual' : [],
                                               'des' : []},
                                 'HL_FOOT' :{'actual' : [],
                                               'des' : []},
                                 'HR_FOOT' :{'actual' : [],
                                               'des' : []},},
                    "MPC PREDICTIONS" : [],
                    
                    "TRACKING PERFORMANCE" : {'actual' : [],
                                             'desired': []
                                              },    
                    "FORCES" : {'FL_FOOT': {'x' : [], 'y' : [], 'z' : []},'FR_FOOT': {'x' : [], 'y' : [], 'z' : []},'HL_FOOT': {'x' : [], 'y' : [], 'z' : []},'HR_FOOT': {'x' : [], 'y' : [], 'z' : []}},

                    "CONTROL EFFORT" : { 'FL_FOOT' : {'FL_HipX' : [],    'FL_HipY': [],     'FL_Knee': []},    
                        'FR_FOOT' : {'FR_HipX' : [],    'FR_HipY': [],     'FR_Knee': []},   
                        'HL_FOOT' : {'HL_HipX' : [],    'HL_HipY': [],     'HL_Knee': []} ,
                        'HR_FOOT' : {'HR_HipX' : [],    'HR_HipY': [],     'HR_Knee': []}}
                                              
                     }
        
       
    def log_feet_data(self, actual, des, leg_name):
        self.log["FEET POS"][leg_name]['actual'].append(actual)
        self.log["FEET POS"][leg_name]['des'].append(des)

    def log_mpc_predictions(self, x_log, x_des, forces_pred, t):
        self.log["MPC PREDICTIONS"].append({'time step' : t,
                                    'predicted_state' : x_log,
                                    'desired_state' : x_des,
                                    'predicted forces' : forces_pred})
        
    def log_tracking_data(self, actual, des):
        self.log[ "TRACKING PERFORMANCE"]['actual'].append(actual)
        self.log[ "TRACKING PERFORMANCE"]['desired'].append(des)


    def save_log(self, filename='simulation_log.pkl'):
        with open(filename, 'wb') as f:
            pickle.dump(self.log, f)


    def load_log(self, filename='simulation_log.pkl'):
        with open(filename, 'rb') as f:
            self.log = pickle.load(f)
        return self.log
    

    def print_log_info(self):
        print(f"""
mpc_freq : {self.log["mpc_freq"]},

sim_params : {self.log["sim_params"]}, 

total_sim_steps : {self.log["total_sim_steps"]}
""")