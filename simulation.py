import numpy as np
import dartpy as dart
import copy
from utils import *
import os
#import ismpc
#import footstep_planner
#import inverse_dynamics as id
#import filter
#import foot_trajectory_generator as ftg
#from logger import Logger

urdf_file_name = "none_string"

class Hrp4Controller(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, hrp4):
        super(Hrp4Controller, self).__init__(world)
        self.world = world
        self.hrp4 = hrp4
        self.time = 0
        self.params = {
            'g': 9.81,
            'h': 0.72,
            'foot_size': 0.1,
            'step_height': 0.02,
            'ss_duration': 70,
            'ds_duration': 30,
            'world_time_step': world.getTimeStep(),
            'first_swing': 'rfoot',
            'µ': 0.5,
            'N': 100,
            'dof': self.hrp4.getNumDofs(),
        }
        self.params['eta'] = np.sqrt(self.params['g'] / self.params['h'])

        # robot links
        if urdf_file_name == "hrp4":
            self.lsole = hrp4.getBodyNode('l_sole')
            self.rsole = hrp4.getBodyNode('r_sole')
            self.torso = hrp4.getBodyNode('torso')
            self.base  = hrp4.getBodyNode('body')
        if urdf_file_name == "cheetah":
            self.lf_sole = hrp4.getBodyNode('LF3')
            self.rf_sole = hrp4.getBodyNode('RF3')
            self.lb_sole = hrp4.getBodyNode('LB3')
            self.rb_sole = hrp4.getBodyNode('RB3')
            self.base  = hrp4.getBodyNode('BODY')


        for i in range(hrp4.getNumJoints()):
            joint = hrp4.getJoint(i)
            dim = joint.getNumDofs()

            # set floating base to passive, everything else to torque
            if   dim == 6: joint.setActuatorType(dart.dynamics.ActuatorType.PASSIVE)
            elif dim == 1: joint.setActuatorType(dart.dynamics.ActuatorType.FORCE)

        # set initial configuration
        initial_configuration = {   'LF_JOINT1': 0.,    'LF_JOINT2': 0.,    'LF_JOINT3': 0.,    \
                                        'RF_JOINT1': 0.,    'RF_JOINT2': 0.,    'RF_JOINT3': 0.,   \
                                        'LB_JOINT1': 0.,    'LB_JOINT2': 0.,    'LB_JOINT3': 0.,    \
                                        'RB_JOINT1': 0.,    'RB_JOINT2': 0.,    'RB_JOINT3': 0.}#, "fixed": 0.}


        for joint_name, value in initial_configuration.items():
            self.hrp4.setPosition(self.hrp4.getDof(joint_name).getIndexInSkeleton(), value * np.pi / 180.)
        
        # position the robot on the ground
        if urdf_file_name == 'hrp4':
            lsole_pos = self.lsole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
            rsole_pos = self.rsole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
            self.hrp4.setPosition(3, - (lsole_pos[0] + rsole_pos[0]) / 2.)
            self.hrp4.setPosition(4, - (lsole_pos[1] + rsole_pos[1]) / 2.)
            self.hrp4.setPosition(5, - (lsole_pos[2] + rsole_pos[2]) / 2.)

        if urdf_file_name == 'cheetah':
            lf_sole_pos = self.lf_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
            rf_sole_pos = self.rf_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
            lb_sole_pos = self.lb_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
            rb_sole_pos = self.rb_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()

            #feet_center = (lf_sole_pos +  rf_sole_pos + lb_sole_pos + rb_sole_pos)/4. 
            # Ochio perche a seconda di come setti initial_config 
            # li ho printati nella config tutto 0 e usiamo quelli fissati per ogni inital_config
            self.hrp4.setPosition(2, -3.14/4)
            self.hrp4.setPosition(3, -0.11352459)
            self.hrp4.setPosition(4, -0.09843932)
            self.hrp4.setPosition(5, -2*-0.1332942308122634)



        # initialize state
        #self.initial = self.retrieve_state()
        #self.contact = 'lfoot' if self.params['first_swing'] == 'rfoot' else 'rfoot' # there is a dummy footstep
        #self.desired = copy.deepcopy(self.initial)

        # selection matrix for redundant dofs
        '''
        redundant_dofs = [ \
            "NECK_Y", "NECK_P", \
            "R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y", "R_ELBOW_P", \
            "L_SHOULDER_P", "L_SHOULDER_R", "L_SHOULDER_Y", "L_ELBOW_P"]
        
        
        # initialize inverse dynamics
        self.id = id.InverseDynamics(self.hrp4, redundant_dofs)

        # initialize footstep planner
        reference = [(0.1, 0., 0.2)] * 5 + [(0.1, 0., -0.1)] * 10 + [(0.1, 0., 0.)] * 10
        self.footstep_planner = footstep_planner.FootstepPlanner(
            reference,
            self.initial['lfoot']['pos'],
            self.initial['rfoot']['pos'],
            self.params
            )

        # initialize MPC controller
        self.mpc = ismpc.Ismpc(
            self.initial, 
            self.footstep_planner, 
            self.params
            )

        # initialize foot trajectory generator
        self.foot_trajectory_generator = ftg.FootTrajectoryGenerator(
            self.initial, 
            self.footstep_planner, 
            self.params
            )

        # initialize kalman filter
        A = np.identity(3) + self.params['world_time_step'] * self.mpc.A_lip
        B = self.params['world_time_step'] * self.mpc.B_lip
        d = np.zeros(9)
        d[7] = - self.params['world_time_step'] * self.params['g']
        H = np.identity(3)
        Q = block_diag(1., 1., 1.)
        R = block_diag(1e1, 1e2, 1e4)
        P = np.identity(3)
        x = np.array([self.initial['com']['pos'][0], self.initial['com']['vel'][0], self.initial['zmp']['pos'][0], \
                      self.initial['com']['pos'][1], self.initial['com']['vel'][1], self.initial['zmp']['pos'][1], \
                      self.initial['com']['pos'][2], self.initial['com']['vel'][2], self.initial['zmp']['pos'][2]])
        self.kf = filter.KalmanFilter(block_diag(A, A, A), \
                                      block_diag(B, B, B), \
                                      d, \
                                      block_diag(H, H, H), \
                                      block_diag(Q, Q, Q), \
                                      block_diag(R, R, R), \
                                      block_diag(P, P, P), \
                                      x)
	    '''
        # initialize logger and plots
        #self.logger = Logger(self.initial)
        #self.logger.initialize_plot(frequency=10)
        
    def customPreStep(self):
        return
        # create current and desired states
        self.current = self.retrieve_state()

        '''
        # update kalman filter
        u = np.array([self.desired['zmp']['vel'][0], self.desired['zmp']['vel'][1], self.desired['zmp']['vel'][2]])
        self.kf.predict(u)
        x_flt, _ = self.kf.update(np.array([self.current['com']['pos'][0], self.current['com']['vel'][0], self.current['zmp']['pos'][0], \
                                            self.current['com']['pos'][1], self.current['com']['vel'][1], self.current['zmp']['pos'][1], \
                                            self.current['com']['pos'][2], self.current['com']['vel'][2], self.current['zmp']['pos'][2]]))
        
        # update current state using kalman filter output
        self.current['com']['pos'][0] = x_flt[0]
        self.current['com']['vel'][0] = x_flt[1]
        self.current['zmp']['pos'][0] = x_flt[2]
        self.current['com']['pos'][1] = x_flt[3]
        self.current['com']['vel'][1] = x_flt[4]
        self.current['zmp']['pos'][1] = x_flt[5]
        self.current['com']['pos'][2] = x_flt[6]
        self.current['com']['vel'][2] = x_flt[7]
        self.current['zmp']['pos'][2] = x_flt[8]

        # get references using mpc
        lip_state, contact = self.mpc.solve(self.current, self.time)

        self.desired['com']['pos'] = lip_state['com']['pos']
        self.desired['com']['vel'] = lip_state['com']['vel']
        self.desired['com']['acc'] = lip_state['com']['acc']
        self.desired['zmp']['pos'] = lip_state['zmp']['pos']
        self.desired['zmp']['vel'] = lip_state['zmp']['vel']

        # get foot trajectories
        feet_trajectories = self.foot_trajectory_generator.generate_feet_trajectories_at_time(self.time)
        for foot in ['lfoot', 'rfoot']:
            for key in ['pos', 'vel', 'acc']:
                self.desired[foot][key] = feet_trajectories[foot][key]

        # set torso and base references to the average of the feet
        for link in ['torso', 'base']:
            for key in ['pos', 'vel', 'acc']:
                self.desired[link][key] = (self.desired['lfoot'][key][:3] + self.desired['rfoot'][key][:3]) / 2.

        # get torque commands using inverse dynamics
        commands = self.id.get_joint_torques(self.desired, self.current, contact) 
        
        # set acceleration commands
        for i in range(self.params['dof'] - 6):
            self.hrp4.setCommand(i + 6, commands[i])
        '''
        # log and plot
        #self.logger.log_data(self.current, self.desired)
        #self.logger.update_plot(self.time)

        self.time += 1

    def retrieve_state(self):
        # com and torso pose (orientation and position)
        com_position = self.hrp4.getCOM()
        #torso_orientation = get_rotvec(self.hrp4.getBodyNode('torso').getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).rotation())
        base_orientation  = get_rotvec(self.hrp4.getBodyNode('BODY' ).getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).rotation())

        # feet poses (orientation and position)
        #l_foot_transform = self.lsole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        #l_foot_orientation = get_rotvec(l_foot_transform.rotation())
        #l_foot_position = l_foot_transform.translation()
        #left_foot_pose = np.hstack((l_foot_orientation, l_foot_position))
        #r_foot_transform = self.rsole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        #r_foot_orientation = get_rotvec(r_foot_transform.rotation())
        #r_foot_position = r_foot_transform.translation()
        #right_foot_pose = np.hstack((r_foot_orientation, r_foot_position))

        lf_foot_transform = self.lf_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        lf_foot_orientation = get_rotvec(lf_foot_transform.rotation())
        lf_foot_position = lf_foot_transform.translation()
        left_front_foot_pose = np.hstack((lf_foot_orientation, lf_foot_position))

        rf_foot_transform = self.rf_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        rf_foot_orientation = get_rotvec(rf_foot_transform.rotation())
        rf_foot_position = rf_foot_transform.translation()
        right_front_foot_pose = np.hstack((rf_foot_orientation, rf_foot_position))

        lb_foot_transform = self.lb_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        lb_foot_orientation = get_rotvec(lb_foot_transform.rotation())
        lb_foot_position = lb_foot_transform.translation()
        left_back_foot_pose = np.hstack((lb_foot_orientation, lb_foot_position))

        rb_foot_transform = self.rb_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        rb_foot_orientation = get_rotvec(rb_foot_transform.rotation())
        rb_foot_position = rb_foot_transform.translation()
        right_back_foot_pose = np.hstack((rb_foot_orientation, rb_foot_position))

        # velocities
        com_velocity = self.hrp4.getCOMLinearVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        #torso_angular_velocity = self.hrp4.getBodyNode('torso').getAngularVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        base_angular_velocity = self.hrp4.getBodyNode('BODY').getAngularVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        #l_foot_spatial_velocity = self.lsole.getSpatialVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        #r_foot_spatial_velocity = self.rsole.getSpatialVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        lf_foot_spatial_velocity = self.lf_sole.getSpatialVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        rf_foot_spatial_velocity = self.rf_sole.getSpatialVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        lb_foot_spatial_velocity = self.lb_sole.getSpatialVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
        rb_foot_spatial_velocity = self.rb_sole.getSpatialVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())


        # compute total contact force
        force = np.zeros(3)
        for contact in world.getLastCollisionResult().getContacts():
            force += contact.force

        # compute zmp
        zmp = np.zeros(3)
        zmp[2] = com_position[2] - force[2] / (self.hrp4.getMass() * self.params['g'] / self.params['h'])
        for contact in world.getLastCollisionResult().getContacts():
            if contact.force[2] <= 0.1: continue
            zmp[0] += (contact.point[0] * contact.force[2] / force[2] + (zmp[2] - contact.point[2]) * contact.force[0] / force[2])
            zmp[1] += (contact.point[1] * contact.force[2] / force[2] + (zmp[2] - contact.point[2]) * contact.force[1] / force[2])

        if force[2] <= 0.1: # threshold for when we lose contact
            zmp = np.array([0., 0., 0.]) # FIXME: this should return previous measurement
        else:
            # sometimes we get contact points that dont make sense, so we clip the ZMP close to the robot
            #midpoint = (l_foot_position + l_foot_position) / 2.
            midpoint = (lf_foot_position + rf_foot_position + lb_foot_position + rb_foot_position) / 4.
            zmp[0] = np.clip(zmp[0], midpoint[0] - 0.3, midpoint[0] + 0.3)
            zmp[1] = np.clip(zmp[1], midpoint[1] - 0.3, midpoint[1] + 0.3)
            zmp[2] = np.clip(zmp[2], midpoint[2] - 0.3, midpoint[2] + 0.3)

        # create state dict
        return {
            #'lfoot': {'pos': left_foot_pose,
            #          'vel': l_foot_spatial_velocity,
            #          'acc': np.zeros(6)},
            #'rfoot': {'pos': right_foot_pose,
            #          'vel': r_foot_spatial_velocity,
            #          'acc': np.zeros(6)},
            'lf_foot': {'pos': left_front_foot_pose, 
                        'vel': lf_foot_spatial_velocity, 
                        'acc': np.zeros(6)},
            'lb_foot': {'pos': left_back_foot_pose, 
                        'vel': lb_foot_spatial_velocity, 
                        'acc': np.zeros(6)},
            'rf_foot': {'pos': right_front_foot_pose, 
                        'vel': rf_foot_spatial_velocity, 
                        'acc': np.zeros(6)},
            'rb_foot': {'pos': right_back_foot_pose, 
                        'vel': rb_foot_spatial_velocity, 
                        'acc': np.zeros(6)},
            'com'  : {'pos': com_position,
                      'vel': com_velocity,
                      'acc': np.zeros(3)},
            #'torso': {'pos': torso_orientation,
            #          'vel': torso_angular_velocity,
            #          'acc': np.zeros(3)},
            'base' : {'pos': base_orientation,
                      'vel': base_angular_velocity,
                      'acc': np.zeros(3)},
            'joint': {'pos': self.hrp4.getPositions(),
                      'vel': self.hrp4.getVelocities(),
                      'acc': np.zeros(self.params['dof'])},
            'zmp'  : {'pos': zmp,
                      'vel': np.zeros(3),
                      'acc': np.zeros(3)}
        }

if __name__ == "__main__":
    world = dart.simulation.World()

    urdfParser = dart.utils.DartLoader()
    current_dir = os.path.dirname(os.path.abspath(__file__))

    urdf_file_name = "cheetah" # cheetah, hrp4
    print(f"ROBOT: {urdf_file_name}")

    hrp4   = urdfParser.parseSkeleton(os.path.join(current_dir, "urdf", f"{urdf_file_name}.urdf"))
    ground = urdfParser.parseSkeleton(os.path.join(current_dir, "urdf", "ground.urdf"))
    world.addSkeleton(hrp4)
    world.addSkeleton(ground)
    world.setGravity([0, 0, -9.81])
    world.setTimeStep(0.01)

    # set default inertia
    default_inertia = dart.dynamics.Inertia(1e-8, np.zeros(3), 1e-10 * np.identity(3))
    for body in hrp4.getBodyNodes():
        if body.getMass() == 0.0:
            body.setMass(1e-8)
            body.setInertia(default_inertia)

    node = Hrp4Controller(world, hrp4)

    # create world node and add it to viewer
    viewer = dart.gui.osg.Viewer()
    node.setTargetRealTimeFactor(10) # speed up the visualization by 10x
    viewer.addWorldNode(node)

    # Show a possible trajectory
    position =[0.8,0.8,1]
    for i in range(7):
        position[1] += 0.1
        display_marker(ground, 'ground_link', 
                    position_in_world_coords=position,
                    color=[255,0,0])
        
    # Show where is the origin of the ground
    display_marker(ground, 'ground_link', 
                    position_in_world_coords=[0,0,0.5],
                    color=[0,255,255])#,
                    #print_bodieds_of_the_object=True)



    #viewer.setUpViewInWindow(0, 0, 1920, 1080)
    #viewer.setUpViewInWindow(0, 0, 1280, 720)
    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([5., -1., 1.5],
                                 [1.,  0., 0.5],
                                 [0.,  0., 1. ])

    
   
    viewer.run()
