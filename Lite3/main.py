import dartpy as dart
import os
from footstep_planner import FootstepPlanner
from foot_trajectory_generator import FootTrajectoryGenerator
import numpy as np
import copy
from utils import *
from single_leg_controller import SingleLegController

class Lite3Controller(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, lite3):
        super(Lite3Controller, self).__init__(world)
        self.world = world
        self.lite3 = lite3
        self.time = 0
        self.params = {
            'g': -9.81,
            'h': 0.72,
            'foot_size': 0.1,   #non serve
            'step_height': 0.01,
            'ss_duration': 100,
            'ds_duration': 100,
            'world_time_step': world.getTimeStep(), # 0.01
            'first_swing': np.array([0,1,1,0]),
            'µ': 0.5,
            'N': 200,
            'dof': self.lite3.getNumDofs(), # 18
            'v_com_ref' : np.array([0.1,0,0])
        }
        self.params['eta'] = np.sqrt(self.params['g'] / self.params['h'])

        self.fl_sole = lite3.getBodyNode('FL_FOOT')
        self.fr_sole = lite3.getBodyNode('FR_FOOT')
        self.hl_sole = lite3.getBodyNode('HL_FOOT')
        self.hr_sole = lite3.getBodyNode('HR_FOOT')
        self.base  = lite3.getBodyNode('TORSO')

        for i in range(lite3.getNumJoints()):
            joint = lite3.getJoint(i)
            dim = joint.getNumDofs()

            # set floating base to passive, everything else to torque
            if   dim == 6: joint.setActuatorType(dart.dynamics.ActuatorType.PASSIVE)
            elif dim == 1: joint.setActuatorType(dart.dynamics.ActuatorType.FORCE)

        # set initial configuration
        initial_configuration = {   'FL_HipX': 0.,    'FL_HipY': 0.,     'FL_Knee': 0,    \
                                    'FR_HipX': 0.,    'FR_HipY': 0.,     'FR_Knee': 0.,   \
                                    'HL_HipX': 0.,    'HL_HipY': 0.,     'HL_Knee': 0.,    \
                                    'HR_HipX': 0.,    'HR_HipY': 0.,     'HR_Knee': 0.}#, "fixed": 0.}
        self.dq = {   'FL_FOOT': [0,0,0], 
                 'FR_FOOT': [0,0,0], 
                 'HL_FOOT': [0,0,0], 
                 'HR_FOOT': [0,0,0] }

        for joint_name, value in initial_configuration.items():
            self.lite3.setPosition(self.lite3.getDof(joint_name).getIndexInSkeleton(), value * np.pi / 180.)

        self.lite3.setPosition(5, 0.43)

        self.fl_sole_pos = self.retrieve_state()['FL_FOOT']['pos'][3:] #self.fl_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
        self.fr_sole_pos = self.retrieve_state()['FR_FOOT']['pos'][3:] #self.fr_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
        self.hl_sole_pos = self.retrieve_state()['HL_FOOT']['pos'][3:] #self.hl_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
        self.hr_sole_pos = self.retrieve_state()['HR_FOOT']['pos'][3:] #self.hr_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()

        self.initial = {
            'FL_FOOT' : self.fl_sole_pos,
            'FR_FOOT' : self.fr_sole_pos,
            'HL_FOOT' : self.hl_sole_pos,
            'HR_FOOT' : self.hr_sole_pos,
            'theta': 0
        }

        #print(self.fl_sole_pos)
        #print(self.fr_sole_pos)
        #print(self.hl_sole_pos)
        #print(self.hr_sole_pos)

        self.footstep_planner = FootstepPlanner(
            vref = np.array([self.params['v_com_ref'][0], self.params['v_com_ref'][1], 0.0]),
            initial_configuration = self.initial,
            leg_displacement_x = 0.010,
            params = self.params,
            )
        
        self.trajectory_generator = FootTrajectoryGenerator(
            footstep_planner = self.footstep_planner,
            params = self.params
        )

        self.leg_controller = SingleLegController(
            lite3 = self.lite3, 
            lite3_controller = self, 
            trajectory_generator =  self.trajectory_generator,
            params = self.params
        )

        #self.trajectory_generator.show_trajectory(foot_to_sample='FR_FOOT', t_start=0, t_end=800, string_axs='z')

    def customPreStep(self):
        # print("-------------------------")
        # collision_result = self.world.getLastCollisionResult()
        # contacts = collision_result.getContacts()
        
        #foot_names = ['FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT']
        # foot_forces = {name: np.zeros(3) for name in foot_names}
    
        # for contact in contacts:
        #     #print()
        #     body1 = contact.collisionObject1.getShapeFrame().getBodyNode()
        #     body2 = contact.collisionObject2.getShapeFrame().getBodyNode()
        #     #print()

        #     #print(body2.getName())
            
        #     for foot in foot_names:
        #         if body1 and body1.getName().startswith(foot.split("_")[0]) == foot:
        #             foot_forces[foot] += contact.force
        #         elif body2 and body2.getName() == foot:
        #             foot_forces[foot] += contact.force

                #print(contact.force)
            #print(foot_forces)
            
        #Test value for controls
        foot_forces = {'FL_FOOT' : [0.0, 0.0, -60.0],
                       'FR_FOOT' : [0.0, 0.0, -60.0],
                       'HL_FOOT' : [0.0, 0.0, -60.0],
                       'HR_FOOT' : [0.0, 0.0, -20.0],
                        }
        
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

        j=0
        for leg_name in tasks:
            if gait[j] == 1:
                tau_curr = self.leg_controller.ground_controller(leg_name, foot_forces[leg_name])
            else:
                tau_curr = self.leg_controller.swing_leg_controller(leg_name)
            tau[leg_name] = tau_curr
            j+=1
        #print(tau)
        for task,value in joint_name.items():
            lite3.setCommand(lite3.getDof(value[0]).getIndexInSkeleton(), tau[task][0]) #non sò se funziona come scrittura per ottenere i valori dell'array
            lite3.setCommand(lite3.getDof(value[1]).getIndexInSkeleton(), tau[task][1])
            lite3.setCommand(lite3.getDof(value[2]).getIndexInSkeleton(), tau[task][2])

        self.time +=1
        return
        
    def retrieve_state(self):

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
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(current_dir, "lite3_urdf/Lite3/urdf", "Lite3.urdf")
    ground_path = os.path.join(current_dir, "lite3_urdf/Lite3/urdf", "ground.urdf")
    urdfParser = dart.utils.DartLoader()

    # Controlla se il file URDF esiste
    if not os.path.exists(urdf_path):
        print(f"Errore: il file {urdf_path} non esiste!")
        exit(-1)
        
    # Creazione del mondo e caricamento del robot
    world = dart.simulation.World()
    lite3 = urdfParser.parseSkeleton(urdf_path)
    ground = urdfParser.parseSkeleton(ground_path)
    world.addSkeleton(lite3)
    world.addSkeleton(ground)
    world.setTimeStep(0.01)


    #print("world variable:")           #ma che è?
    #print(dir(world))
    #print("---\nlite3 variable:")
    #print(dir(lite3))
    #print("---\ngroun variable:")
    #print(dir(ground))
              
    num_joints =lite3.getNumJoints()
    
    inertia_matrix = lite3.getMassMatrix()[3:6]

    # set default inertia
    default_inertia = dart.dynamics.Inertia(1e-8, np.zeros(3), 1e-10 * np.identity(3))
    for body in lite3.getBodyNodes():
        #print(body.getName())
        if body.getMass() == 0.0:
            body.setMass(1e-8)
            body.setInertia(default_inertia)

    node = Lite3Controller(world, lite3)
    world.setGravity([0, 0, node.params['g']])
    
    display_marker(ground, 'ground_link', position_in_world_coords=[0,0,0.5],
                color= [255, 255, 255], print_bodieds_of_the_object=False)
    # display_marker(ground, 'ground_link', position_in_world_coords=[0.1,0,0.5],
    #             color= [0, 255, 0], print_bodieds_of_the_object=False)
    # display_marker(ground, 'ground_link', position_in_world_coords=[0,0.1,0.5],
    #             color= [255, 0, 0], print_bodieds_of_the_object=False)
    # display_marker(ground, 'ground_link', position_in_world_coords=[0,0,0.6],
    #             color= [0, 0, 255], print_bodieds_of_the_object=False)
    
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
