import dartpy as dart
import numpy as np
from utils import *

class LegController:
    def __init__(self, lite3, lite3_controller, params):
        self.lite3 = lite3
        self.lite3_controller = lite3_controller
        self.params = params

        self.Kp = np.eye(3)*8
        self.Kd = np.eye(3)*8

    def ground_controller(self, leg_name, forces):
        J = {
            'FL_FOOT' : self.lite3.getLinearJacobian(self.lite3_controller.fl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,6:9],
            'FR_FOOT' : self.lite3.getLinearJacobian(self.lite3_controller.fr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,9:12],
            'HL_FOOT' : self.lite3.getLinearJacobian(self.lite3_controller.hl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,12:15],
            'HR_FOOT' : self.lite3.getLinearJacobian(self.lite3_controller.hr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,15:],
            }
        
        #print(f"{leg_name} -> {forces[leg_name]}")
        tau = J[leg_name].T @ forces[leg_name]
        #print(tau)
        return tau

    def swing_controller(self, leg_name, p_ref, v_ref, a_ref, dqi):
        '''
        - leg_name: id del piede
        - p_ref, v_ref: are the corresponding references for the position and velocity of the swing leg trajectory
        - a_ref: reference acceleration in the body frame
        - dqi: join velocity

        p_ref, v_ref, a_ref: dovrebbero corrispondere agli input dati al trajectory_generator
                             i.e. sono v_ref = 0, p_ref = target_pos = np.array(self.plan[step_index]['pos'][foot])

        dqi: dovrebbe venire dallo swing_data di generate_feet_trajectories_at_time (?)
             i.e. generate_feet_trajectories_at_time(....)['vel'][3:]

        '''

        # Computing Jacobian, Time Jacobian and Inertia Matrix
        J = {
            'FL_FOOT' : self.lite3.getLinearJacobian(self.lite3_controller.fl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,6:9],
            'FR_FOOT' : self.lite3.getLinearJacobian(self.lite3_controller.fr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,9:12],
            'HL_FOOT' : self.lite3.getLinearJacobian(self.lite3_controller.hl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,12:15],
            'HR_FOOT' : self.lite3.getLinearJacobian(self.lite3_controller.hr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,15:],
            }
        
        Jdot = {
            'FL_FOOT' : self.lite3.getJacobianClassicDeriv(self.lite3_controller.fl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[3:,6:9],
            'FR_FOOT' : self.lite3.getJacobianClassicDeriv(self.lite3_controller.fr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[3:,9:12],
            'HL_FOOT' : self.lite3.getJacobianClassicDeriv(self.lite3_controller.hl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[3:,12:15],
            'HR_FOOT' : self.lite3.getJacobianClassicDeriv(self.lite3_controller.hr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[3:,15:],
            }
        
        inertia_matrix = self.lite3.getMassMatrix()[3:6]

        M = {
            'FL_FOOT' : inertia_matrix[:,6:9],
            'FR_FOOT' : inertia_matrix[:,9:12],
            'HL_FOOT' : inertia_matrix[:,12:15],
            'HR_FOOT' : inertia_matrix[:,15:],
            }
        
        coriolis_gravity = self.lite3.getCoriolisAndGravityForces()

        CG = {
            'FL_FOOT' : coriolis_gravity[6:9],
            'FR_FOOT' : coriolis_gravity[9:12],
            'HL_FOOT' : coriolis_gravity[12:15],
            'HR_FOOT' : coriolis_gravity[15:],
        }

        # Extracting Jacobian, Time Jacobian and Inertia Matrix of a single leg
        Ji = J[leg_name]
        Jdoti = Jdot[leg_name]
        Mi = M[leg_name]

        op_space_mi = Ji*Mi*Ji.transpose()

        # Retreving state information to compute torque
        current_state = self.lite3_controller.retrieve_state()
        pi = current_state[leg_name]['pos'][3:]
        vi = current_state[leg_name]['vel'][3:]

        # Computing torque of the leg

        # Per matrix C e G, dovrebbe esserci 
        # getCoriolisAndGravityForces ( inverse dynamics humanoid )
        # ma è un vettore riga dim = 18
        # per la velocità del giunTo, dovrebbe essere data dal trajectory
        
        tau_coriolis_gravity = CG[leg_name]
        tau_ff = Ji.transpose() @ op_space_mi @ ( a_ref - Jdoti @ dqi) + tau_coriolis_gravity
        tau = Ji.transpose() @ ( self.Kp @ (p_ref - pi) + self.Kd @ (v_ref - vi) ) + tau_ff
        
        return tau
    