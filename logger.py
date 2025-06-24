import numpy as np
from matplotlib import pyplot as plt
import pickle

class Logger():
    """initial: 
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
                    "FEET POS Z" : { 'FL_FOOT' : {'actual' : [],                # correggi sono x y z non solo z
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
        self.log["FEET POS Z"][leg_name]['actual'].append(actual)
        self.log["FEET POS Z"][leg_name]['des'].append(des)

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



# class Logger():
#     def __init__(self, initial):
#         self.log = {}
#         for item in initial:
#             self.log[item] = []

#     def log_data(self, plot_name, plot_data):
#         if plot_name not in self.log.keys():
#             return
#         self.log[plot_name].append(plot_data)

#     def initialize_plot_group(self, frequency=1):
#         self.frequency = frequency
#         #self.keys_to_plot = keys_to_plot

#         # Raggruppa per prefisso (es. com, torque, etc.)
#         grouped_keys = {}
#         for key in self.log.keys():
#             prefix = key.split("_")[0]
#             grouped_keys.setdefault(prefix, []).append(key)

#         self.grouped_keys = grouped_keys

#         self.fig, self.ax = plt.subplots(len(grouped_keys), 1, figsize=(6, 4 * len(grouped_keys)))
#         if len(grouped_keys) == 1:
#             self.ax = [self.ax]  # ensure list-like

#         self.lines = {}
#         for i, (prefix, keys) in enumerate(grouped_keys.items()):
#             for key in keys:
#                 self.lines[key], = self.ax[i].plot([], [], label=key)
#             self.ax[i].set_title(prefix)
#             self.ax[i].legend()
#             self.ax[i].grid(True)

#         plt.ion()
#         plt.tight_layout()
#         plt.show()

#     def update_plot_group(self, time):
#         if time % self.frequency != 0:
#             return

#         # Raggruppa chiavi per prefisso (prima parte, separata da "_")
#         grouped_keys = {}
#         #for key in self.keys_to_plot:
#         for key in self.log.keys():
#             prefix = key.split("_")[0]
#             grouped_keys.setdefault(prefix, []).append(key)

#         for i, (prefix, keys) in enumerate(grouped_keys.items()):
#             for key in keys:
#                 if key not in self.log:
#                     continue
#                 data = self.log[key]
#                 self.lines[key].set_data(np.arange(len(data)), data)

#             self.ax[i].relim()
#             self.ax[i].autoscale_view()

#         self.fig.canvas.draw()
#         self.fig.canvas.flush_events()

#     def initialize_plot(self, frequency=1):
#         self.frequency = frequency

#         self.fig, self.ax = plt.subplots(len(self.log.keys()), 1, figsize=(6, 4 * len(keys_to_plot)))

#         if len(self.log.keys()) == 1:
#             self.ax = [self.ax]  # Ensure list-like structure

#         self.lines = {}
#         for i, key in enumerate(self.log.keys()):
#             self.lines[key], = self.ax[i].plot([], [], label=key)
#             self.ax[i].set_title(key)
#             self.ax[i].legend()
#             self.ax[i].grid(True)

#         plt.ion()
#         plt.tight_layout()
#         plt.show()

#     def update_plot(self, time):
#         if time % self.frequency != 0:
#             return

#         for i, key in enumerate(self.keys_to_plot):
#             if key not in self.log:
#                 continue
#             data = self.log[key]
#             self.lines[key].set_data(np.arange(len(data)), data)
#             self.ax[i].relim()
#             self.ax[i].autoscale_view()

#         self.fig.canvas.draw()
#         self.fig.canvas.flush_events()
