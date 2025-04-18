import dartpy as dart
import numpy as np
from utils import *

class SingleLegController():
    def __init__(self, lite3, lite3_controller, trajectory_generator, params):
        self.lite3 = lite3
        self.lite3_controller = lite3_controller
        self.trajectory_generator = trajectory_generator
        self.params = params

        self.Kp = np.eye(3)*0.8
        self.Kd = np.eye(3)*0.8

    def ground_controller(self, leg_name, forces): #forces will be determined by the MPC

        J = {
            'FL_FOOT' : self.lite3.getLinearJacobian(self.lite3_controller.fl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,6:9],
            'FR_FOOT' : self.lite3.getLinearJacobian(self.lite3_controller.fr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,9:12],
            'HL_FOOT' : self.lite3.getLinearJacobian(self.lite3_controller.hl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,12:15],
            'HR_FOOT' : self.lite3.getLinearJacobian(self.lite3_controller.hr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,15:],
            }
        
        tau = J[leg_name].T @ forces
        return tau
    
    def swing_leg_controller(self, leg_name):

        swing_data = self.trajectory_generator.generate_feet_trajectories_at_time(self.lite3_controller.time, leg_name)
        p_des = swing_data['pos'][3:]
        v_des = swing_data['vel'][3:]
        a_des = swing_data['acc'][3:]

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

        CG = {                                          #se non funziona, check
            'FL_FOOT' : coriolis_gravity[6:9],
            'FR_FOOT' : coriolis_gravity[9:12],
            'HL_FOOT' : coriolis_gravity[12:15],
            'HR_FOOT' : coriolis_gravity[15:],
        }

        J_leg = J[leg_name]
        J_leg_dot = Jdot[leg_name]
        M_leg = M[leg_name]

        op_space_mi = J_leg*M_leg*J_leg.transpose()

        current_state = self.lite3_controller.retrieve_state()
        p_leg_curr = current_state[leg_name]['pos'][3:]
        v_leg_curr= current_state[leg_name]['vel'][3:]

        tau_coriolis_gravity = CG[leg_name]
        tau_ff = J_leg.transpose() @ op_space_mi @ ( a_des - J_leg_dot @ self.lite3_controller.dq[leg_name]) + tau_coriolis_gravity
        tau = J_leg.transpose() @ ( self.Kp @ ( p_des - p_leg_curr) + self.Kd @ (v_des - v_leg_curr) ) + tau_ff
        return tau






