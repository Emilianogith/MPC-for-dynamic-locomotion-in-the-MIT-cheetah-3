import dartpy as dart
import os
from footstep_planner import FootstepPlanner
from foot_trajectory_generator import FootTrajectoryGenerator
import numpy as np
import time
from utils import *
from logger import Logger
from mpc import MPC


class Lite3Controller(dart.gui.osg.RealTimeWorldNode):
    """This controller manages the robot's behavior during the simulation. It interacts with 
        the simulation environment to retrieve the robot's state and perform actions at each step.


            Main methods:
            - CustomPreStep(): implements a customization of the simulation built-in method. 
                               Core function in the simulation.
            - retrieve_state(): returns the current information about the state from the simulation.
            - ground_controller(): compute the torques based on the 3R robot model of the legs that generate the specific e-e forces.
            - swing_leg_controller(): implements a controller for the swinging legs that track the trajoctory of the planned step.
    """

    def __init__(self, world, ground, lite3):
        super(Lite3Controller, self).__init__(world)
        self.world = world
        self.lite3 = lite3
        self.time = 0
        self.ground = ground
        self.params = {
            'g': -9.81,
            'h': 0.285, #0.315, #0.285
            'step_height': 0.08,
            'ss_duration': 10,
            'ds_duration': 5,
            'world_time_step': world.getTimeStep(), # 0.01
            'total_steps': 20,
            'first_swing': np.array([0,0,1,1]), 
            'µ': 1,
            'N': 60,
            'dof': self.lite3.getNumDofs(), # 18
            'v_com_ref' : np.array([0.18,0.0,0.0]),
            'theta_dot' : 0.0,
            'log_samples' : 1000
        }

        self.Kp = np.eye(3)*250
        self.Kd = np.eye(3)*15

        self.fl_sole = lite3.getBodyNode('FL_FOOT')
        self.fr_sole = lite3.getBodyNode('FR_FOOT')
        self.hl_sole = lite3.getBodyNode('HL_FOOT')
        self.hr_sole = lite3.getBodyNode('HR_FOOT')
        self.base    = lite3.getBodyNode('TORSO')

        for i in range(lite3.getNumJoints()):
            joint = lite3.getJoint(i)
            dim = joint.getNumDofs()

            # set floating base to passive, everything else to torque
            if   dim == 6: joint.setActuatorType(dart.dynamics.ActuatorType.PASSIVE)
            elif dim == 1: joint.setActuatorType(dart.dynamics.ActuatorType.FORCE)

        # set initial configuration
        initial_configuration = {   'FL_HipX': 0.,    'FL_HipY': -60.,     'FL_Knee': 90.,   \
                                    'FR_HipX': 0.,    'FR_HipY': -60.,     'FR_Knee': 90.,   \
                                    'HL_HipX': 0.,    'HL_HipY': -60.,     'HL_Knee': 90.,   \
                                    'HR_HipX': 0.,    'HR_HipY': -60.,     'HR_Knee': 90.}#, "fixed": 0.}
        
        self.dq = {   'FL_FOOT': [0,0,0], 
                      'FR_FOOT': [0,0,0], 
                      'HL_FOOT': [0,0,0], 
                      'HR_FOOT': [0,0,0] }

        for joint_name, value in initial_configuration.items():
            self.lite3.setPosition(self.lite3.getDof(joint_name).getIndexInSkeleton(), value * np.pi / 180.)
        
        # Hand-tuned robot's spawning 
        #self.lite3.setPosition(5, 0.43)     # Legs straight
        self.lite3.setPosition(5, 0.295+0.004)     # Legs bent   

        initial_state = self.retrieve_state()
        self.fl_sole_pos = initial_state['FL_FOOT']['pos'][3:]
        self.fr_sole_pos = initial_state['FR_FOOT']['pos'][3:]
        self.hl_sole_pos = initial_state['HL_FOOT']['pos'][3:]
        self.hr_sole_pos = initial_state['HR_FOOT']['pos'][3:]
        self.roll        = initial_state['TORSO']['pos'][0] 
        self.pitch       = initial_state['TORSO']['pos'][1] 
        self.yaw         = initial_state['TORSO']['pos'][2] 
        self.com_pos     = initial_state['com']['pos']
        self.initial = {
            'FL_FOOT' : self.fl_sole_pos,
            'FR_FOOT' : self.fr_sole_pos,
            'HL_FOOT' : self.hl_sole_pos,
            'HR_FOOT' : self.hr_sole_pos,
            'roll' : self.roll,
            'pitch': self.pitch,
            'yaw'  : self.yaw,
            'com_position' : self.com_pos,
        }


        self.footstep_planner = FootstepPlanner(
            initial_configuration = self.initial,
            params = self.params,
            )
        
        self.trajectory_generator = FootTrajectoryGenerator(
            footstep_planner = self.footstep_planner,
            params = self.params
        )

        self.mpc = MPC(
            lite3 = self,
            initial = self.initial,
            footstep_planner = self.footstep_planner,
            params = self.params
        )

        # initialize logger and plots
        self.plot_keys = {'params' : self.params,
                         "total_sim_steps" : self.params['log_samples']}
        

        self.logger = Logger(self.plot_keys)
      


    def customPreStep(self):
        """ This method consists in the overwriting of the built-in method to run the simulation ..."""

        tau = { 'FL_FOOT' : [0, 0, 0],
                'FR_FOOT' : [0, 0, 0],
                'HL_FOOT' : [0, 0, 0],
                'HR_FOOT' : [0, 0, 0],
              }
        
        joint_name = {  'FL_FOOT' : ['FL_HipX',    'FL_HipY',     'FL_Knee'],    
                        'FR_FOOT' : ['FR_HipX',    'FR_HipY',     'FR_Knee'],   
                        'HL_FOOT' : ['HL_HipX',    'HL_HipY',     'HL_Knee'],    
                        'HR_FOOT' : ['HR_HipX',    'HR_HipY',     'HR_Knee'],
                        }
        
        tasks = ['FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT']

        for name,value in joint_name.items():
            self.dq[name][0] = self.lite3.getVelocity(self.lite3.getDof(value[0]).getIndexInSkeleton())
            self.dq[name][1] = self.lite3.getVelocity(self.lite3.getDof(value[1]).getIndexInSkeleton())
            self.dq[name][2] = self.lite3.getVelocity(self.lite3.getDof(value[2]).getIndexInSkeleton())

        step_index = self.footstep_planner.get_step_index_at_time(self.time)
        gait = self.footstep_planner.plan[step_index]['feet_id']

        tau_ground = self.ground_controller(self.time)
        for j, leg_name in enumerate(tasks):
            if gait[j] == 1:
                tau[leg_name] = tau_ground[leg_name]
                p_des =  self.footstep_planner.plan[step_index]['pos'][leg_name] # x-Y des

            else:
                tau[leg_name], p_des = self.swing_leg_controller(leg_name)
                

            # log feet data
            state = self.retrieve_state()
            self.logger.log_feet_data(state[leg_name]['pos'][3:6],p_des, leg_name)


        for task,value in joint_name.items():
            lite3.setCommand(lite3.getDof(value[0]).getIndexInSkeleton(), tau[task][0]) 
            lite3.setCommand(lite3.getDof(value[1]).getIndexInSkeleton(), tau[task][1])
            lite3.setCommand(lite3.getDof(value[2]).getIndexInSkeleton(), tau[task][2])

            for n,joint in enumerate(value):
                self.logger.log["CONTROL EFFORT"][task][joint].append(tau[task][n])



        self.logger.log['time array'].append(self.time)
        self.time +=1

        # save logs after total_sim_steps
        if self.time == self.plot_keys['total_sim_steps']:
            self.logger.save_log()
            print('logs saved')
        
        return




    def ground_controller(self, t): 
        start_time = time.time()
        
        forces = self.mpc.solve(t, self.logger)
        
        curr_time = time.time()
        mpc_freq = 1/(curr_time - start_time)
         
        # log the mpc freq
        self.logger.log['mpc_freq'] = (self.logger.log['mpc_freq'] * self.time + mpc_freq )/(self.time +1) # moving average


        J = {
            'FL_FOOT' : self.lite3.getLinearJacobian(self.fl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,6:9],
            'FR_FOOT' : self.lite3.getLinearJacobian(self.fr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,9:12],
            'HL_FOOT' : self.lite3.getLinearJacobian(self.hl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,12:15],
            'HR_FOOT' : self.lite3.getLinearJacobian(self.hr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,15:],
            }
        
        tau = {}
        for leg_name in J.keys():
            tau[leg_name] = J[leg_name].T @ -forces[leg_name]

            self.logger.log['FORCES'][leg_name]['x'].append(forces[leg_name][0])
            self.logger.log['FORCES'][leg_name]['y'].append(forces[leg_name][1])
            self.logger.log['FORCES'][leg_name]['z'].append(forces[leg_name][2])
        return tau
    



    
    def swing_leg_controller(self, leg_name):

        swing_data = self.trajectory_generator.generate_feet_trajectories_at_time(self.time, leg_name)
        p_des = swing_data['pos'][3:]
        v_des = swing_data['vel'][3:]
        a_des = swing_data['acc'][3:]

        if p_des[2]<0:
            p_des[2] = 0
            v_des[2] = 0

        J = {
            'FL_FOOT' : self.lite3.getLinearJacobian(self.fl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,6:9],
            'FR_FOOT' : self.lite3.getLinearJacobian(self.fr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,9:12],
            'HL_FOOT' : self.lite3.getLinearJacobian(self.hl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,12:15],
            'HR_FOOT' : self.lite3.getLinearJacobian(self.hr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,15:],
            }
        
        Jdot = {
            'FL_FOOT' : self.lite3.getJacobianClassicDeriv(self.fl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[3:,6:9],
            'FR_FOOT' : self.lite3.getJacobianClassicDeriv(self.fr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[3:,9:12],
            'HL_FOOT' : self.lite3.getJacobianClassicDeriv(self.hl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[3:,12:15],
            'HR_FOOT' : self.lite3.getJacobianClassicDeriv(self.hr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[3:,15:],
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

        J_leg = J[leg_name]
        J_leg_dot = Jdot[leg_name]
        M_leg = M[leg_name]

        op_space_mi = J_leg*M_leg*J_leg.transpose()

        current_state = self.retrieve_state()
        p_leg_curr = current_state[leg_name]['pos'][3:]
        v_leg_curr= current_state[leg_name]['vel'][3:]

        tau_coriolis_gravity = CG[leg_name]
        tau_ff = J_leg.transpose() @ op_space_mi @ ( a_des - J_leg_dot @ self.dq[leg_name]) + tau_coriolis_gravity
        tau = J_leg.transpose() @ ( self.Kp @ ( p_des - p_leg_curr) + self.Kd @ (v_des - v_leg_curr) ) + tau_ff

        return tau, p_des



    def retrieve_state(self):
        """
        Returns the current state information from the simulation, including:

        - Feet: position, velocity, and acceleration
        - Center of Mass (COM): position, velocity, and acceleration
        - Torso: orientation and angular velocity
        - Joint variables: positions and velocities
        """

        com_position = self.lite3.getCOM()
        torso_orientation  = get_rotvec(self.lite3.getBodyNode('TORSO' ).getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).rotation())

        fl_foot_transform = self.fl_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        fl_foot_orientation = get_rotvec(fl_foot_transform.rotation())
        fl_foot_position = fl_foot_transform.translation()
        front_left_foot_pose = np.hstack((fl_foot_orientation, fl_foot_position))

        fr_foot_transform = self.fr_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        fr_foot_orientation = get_rotvec(fr_foot_transform.rotation())
        fr_foot_position = fr_foot_transform.translation()
        front_right_foot_pose = np.hstack((fr_foot_orientation, fr_foot_position))

        hl_foot_transform = self.hl_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        hl_foot_orientation = get_rotvec(hl_foot_transform.rotation())
        hl_foot_position = hl_foot_transform.translation()
        hip_left_foot_pose = np.hstack((hl_foot_orientation, hl_foot_position))

        hr_foot_transform = self.hr_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        hr_foot_orientation = get_rotvec(hr_foot_transform.rotation())
        hr_foot_position = hr_foot_transform.translation()
        hip_right_foot_pose = np.hstack((hr_foot_orientation, hr_foot_position))

        # velocities
        com_velocity = self.lite3.getCOMLinearVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        base_angular_velocity = self.lite3.getBodyNode('TORSO').getAngularVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        fl_foot_spatial_velocity = self.fl_sole.getSpatialVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        fr_foot_spatial_velocity = self.fr_sole.getSpatialVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        hl_foot_spatial_velocity = self.hl_sole.getSpatialVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        hr_foot_spatial_velocity = self.hr_sole.getSpatialVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())

        # create state dict
        return {
            'FL_FOOT': {'pos': front_left_foot_pose, 
                        'vel': fl_foot_spatial_velocity, 
                        'acc': np.zeros(6)},
            'HL_FOOT': {'pos': hip_left_foot_pose, 
                        'vel': hl_foot_spatial_velocity, 
                        'acc': np.zeros(6)},
            'FR_FOOT': {'pos': front_right_foot_pose, 
                        'vel': fr_foot_spatial_velocity, 
                        'acc': np.zeros(6)},
            'HR_FOOT': {'pos': hip_right_foot_pose, 
                        'vel': hr_foot_spatial_velocity, 
                        'acc': np.zeros(6)},
            'com'  : {'pos': com_position,
                      'vel': com_velocity,
                      'acc': np.zeros(3)},
            'TORSO' : {'pos': torso_orientation,
                      'vel': base_angular_velocity,
                      'acc': np.zeros(3)},
            'joint': {'pos': self.lite3.getPositions(),
                      'vel': self.lite3.getVelocities(),
                      'acc': np.zeros(self.params['dof'])},
        }


if __name__ == "__main__":

    # URDF files loading:
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(current_dir, "lite3_urdf/urdf", "Lite3.urdf")  
    ground_path = os.path.join(current_dir, "lite3_urdf/urdf", "ground.urdf")
    urdfParser = dart.utils.DartLoader()

    if not os.path.exists(urdf_path):
        print(f"Error: file {urdf_path} does not exist!")
        exit(-1)
        
    # Create the simulation world and load the robot model
    world = dart.simulation.World()
    lite3 = urdfParser.parseSkeleton(urdf_path)
    ground = urdfParser.parseSkeleton(ground_path)


    world.addSkeleton(lite3)
    world.addSkeleton(ground)
    world.setTimeStep(0.01) #0.01

              
    num_joints =lite3.getNumJoints()
    inertia_matrix = lite3.getMassMatrix()[3:6]

    total_mass = 0
    # set default inertia
    default_inertia = dart.dynamics.Inertia(1e-8, np.zeros(3), 1e-10 * np.identity(3))
    for body in lite3.getBodyNodes():
        if body.getMass() == 0.0:
            body.setMass(1e-8)
            body.setInertia(default_inertia)
    
    # Initialization of the main node
    node = Lite3Controller(world, ground, lite3)
    world.setGravity([0, 0, node.params['g']])
    
    # plot steps in the simulation
    for step in node.footstep_planner.plan:
        x_hl_foot = step['pos']["HL_FOOT"][0]
        y_hl_foot = step['pos']["HL_FOOT"][1]

        x_hr_foot = step['pos']["HR_FOOT"][0]
        y_hr_foot = step['pos']["HR_FOOT"][1]

        x_fl_foot = step['pos']["FL_FOOT"][0]
        y_fl_foot = step['pos']["FL_FOOT"][1]

        x_fr_foot = step['pos']["FR_FOOT"][0]
        y_fr_foot = step['pos']["FR_FOOT"][1]

        display_marker(ground, 'ground_link', position_in_world_coords=[x_hl_foot,y_hl_foot,0.5],
               color= [255, 0, 0], print_bodieds_of_the_object=False)
        display_marker(ground, 'ground_link', position_in_world_coords=[x_hr_foot,y_hr_foot,0.5],
               color= [0, 0, 255], print_bodieds_of_the_object=False)
        display_marker(ground, 'ground_link', position_in_world_coords=[x_fl_foot,y_fl_foot,0.5],
                color= [0, 255, 0], print_bodieds_of_the_object=False)
        display_marker(ground, 'ground_link', position_in_world_coords=[x_fr_foot,y_fr_foot,0.5],
                color= [255, 0, 255], print_bodieds_of_the_object=False)    


    # create world node and add it to viewer
    viewer = dart.gui.osg.Viewer()
    node.setTargetRealTimeFactor(1) # speed up the visualization by 10x
    viewer.addWorldNode(node)

    viewer.setUpViewInWindow(0, 0, 1270, 720)
    viewer.setCameraHomePosition([5., -1., 1.5],
                                [1.,  0., 0.5],
                                [0.,  0., 1. ])
    
    viewer.run()