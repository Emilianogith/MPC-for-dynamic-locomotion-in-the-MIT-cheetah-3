import dartpy as dart
import os
#from lite3_controller import Lite3Controller
from footstep_planner import FootstepPlanner
import numpy as np
import copy
from utils import *

class Lite3Controller(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, lite3):
        super(Lite3Controller, self).__init__(world)
        self.world = world
        self.lite3 = lite3
        self.time = 0
        self.params = {
            'g': 9.81,
            'h': 0.72,
            'foot_size': 0.1,
            'step_height': 0.02,
            'ss_duration': 70,
            'ds_duration': 30,
            'world_time_step': world.getTimeStep(), # 0.01
            'first_swing': 'rfoot',
            'µ': 0.5,
            'N': 100,
            'dof': self.lite3.getNumDofs(), # 18
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


        for joint_name, value in initial_configuration.items():
            self.lite3.setPosition(self.lite3.getDof(joint_name).getIndexInSkeleton(), value * np.pi / 180.)
        
        #print('totti2')
        self.fl_sole_pos = self.fl_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
        self.fr_sole_pos = self.fr_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
        self.hl_sole_pos = self.hl_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
        self.hr_sole_pos = self.hr_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
        #feet_center = (lf_sole_pos +  rf_sole_pos + lb_sole_pos + rb_sole_pos)/4. 
        # Ochio perche a seconda di come setti initial_config 
        # # li ho printati nella config tutto 0 e usiamo quelli fissati per ogni inital_config
        # self.lite3.setPosition(2, -3.14/4)
        # self.lite3.setPosition(3, -0.11352459)
        # self.lite3.setPosition(4, -0.09843932)
        self.lite3.setPosition(5, 0.43)
        self.it_f = 0

    def customPreStep(self):
        
        J = {'FL_FOOT' : lite3.getLinearJacobian(self.fl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,6:9],
             'FR_FOOT' : lite3.getLinearJacobian(self.fr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,9:12],
             'HL_FOOT' : lite3.getLinearJacobian(self.hl_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,12:15],
             'HR_FOOT' : lite3.getLinearJacobian(self.hr_sole, inCoordinatesOf=dart.dynamics.Frame.World())[:,15:],
             }
        #Test value for controls
        f = {'FL_FOOT' : [0.0, 0.0, -31.0],
             'FR_FOOT' : [0.0, 0.0, -31.0],
             'HL_FOOT' : [0.0, 0.0, -31.0],
             'HR_FOOT' : [0.0, 0.0, -31.0],
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
        
        for task in tasks:
            tau_curr = J[task].T @ f[task]
            tau[task] = tau_curr

        force = np.zeros(3)
        if self.it_f <= -20:
            #print(f"num collision: {len( world.getLastCollisionResult().getContacts())} ")
            for contact in world.getLastCollisionResult().getContacts():
                force += contact.force
                #print(contact.force)

            print(f"Total force: {force}")

            print("-----------------------")

        self.it_f += 1

        # #actual commands
        #print(lite3.getDof(joint_name.get(task)[0]).getIndexInSkeleton())
        #print(tau)
        #print(J)
        #print("tott3")
        
        for task in joint_name:
            lite3.setCommand(lite3.getDof(joint_name.get(task)[0]).getIndexInSkeleton(), tau[task][0]) #non sò se funziona come scrittura per ottenere i valori dell'array
            lite3.setCommand(lite3.getDof(joint_name.get(task)[1]).getIndexInSkeleton(), tau[task][1])
            lite3.setCommand(lite3.getDof(joint_name.get(task)[2]).getIndexInSkeleton(), tau[task][2])
        return
        # # #-------------------------------------------
        v_ref = [0.1,-0.00,0.0]
        q_des = np.linalg.pinv(J['FR_FOOT']) @ v_ref
        i = 0
        for elem in joint_name['FR_FOOT']:
            #lite3.setVelocity(lite3.getDof(elem).getIndexInSkeleton(), q_des[i])
            i+=1


        collision_result = self.world.getLastCollisionResult()
        foot_names = ['FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT']  # Front-right, front-left, rear-right, rear-left

        # Dictionary to store contact forces for each foot
        foot_contact_forces = {foot: None for foot in foot_names}
        #print(collision_result.getContacts())
        #for contact in collision_result.getContacts():
            #print(contact.force)
        #    print(contact.getBodyNode1())
        # # Iterate over contact points
        # for contact in collision_result.getContact():
        #     body_node = contact.bodyNode  # Body involved in contact
        #     force = contact.force  # Contact force vector
        #     point = contact.point  # Contact point position

        #     if body_node and body_node.getName() in foot_names:
        #         foot_contact_forces[body_node.getName()] = force

        # for foot, force in foot_contact_forces.items():
        #     if force is not None:
        #         print(f"Contact force on {foot}: {force}")
        #     else:
        #         print(f"No contact detected for {foot}")


        self.time += 1    




if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(current_dir, "lite3_urdf/Lite3/urdf", "Lite3.urdf")
    ground_path = os.path.join(current_dir, "lite3_urdf/Lite3/urdf", "ground.urdf")
    urdfParser = dart.utils.DartLoader()

    # Controlla se il file URDF esiste
    if not os.path.exists(urdf_path):
        print(f"Errore: il file {urdf_path} non esiste!")
    else:
        # Creazione del mondo e caricamento del robot
        world = dart.simulation.World()
        lite3 = urdfParser.parseSkeleton(urdf_path)
        ground = urdfParser.parseSkeleton(ground_path)
        world.addSkeleton(lite3)
        world.addSkeleton(ground)
        world.setGravity([0, 0, -9.81])
        world.setTimeStep(0.01)


        num_joints =lite3.getNumJoints()
        print(f"Il robot ha {num_joints} giunti:")
        
        #for i in range(num_joints):
            #joint = lite3.getJoint(i)
            #print(joint.getName())
            #print(lite3.getDof(i).getIndexInSkeleton())
            #print(f"- Joint {i}: {joint.getName()} (Tipo: {joint.getType()})")

            #body_node = lite3.getBodyNode(i)
            #print(body_node.getIndexInSkeleton())
            # print(f"- Body Node {i}: {body_node.getName()}")

    # set default inertia
        default_inertia = dart.dynamics.Inertia(1e-8, np.zeros(3), 1e-10 * np.identity(3))
        for body in lite3.getBodyNodes():
            if body.getMass() == 0.0:
                body.setMass(1e-8)
                body.setInertia(default_inertia)

        node = Lite3Controller(world, lite3)

        display_marker(ground, 'ground_link', position_in_world_coords=[0,0,0.5],
                    color= [255, 255, 255], print_bodieds_of_the_object=False)
        # display_marker(ground, 'ground_link', position_in_world_coords=[0.1,0,0.5],
        #             color= [0, 255, 0], print_bodieds_of_the_object=False)
        # display_marker(ground, 'ground_link', position_in_world_coords=[0,0.1,0.5],
        #             color= [255, 0, 0], print_bodieds_of_the_object=False)
        # display_marker(ground, 'ground_link', position_in_world_coords=[0,0,0.6],
        #             color= [0, 0, 255], print_bodieds_of_the_object=False)
        
        lb_foot = node.hl_sole_pos
        lb_foot.shape = (3,1)  

        rb_foot = node.hr_sole_pos
        rb_foot.shape = (3,1)

        lf_foot = node.fl_sole_pos
        lf_foot.shape = (3,1)

        rf_foot = node.fr_sole_pos
        rf_foot.shape = (3,1)

        v_com = np.array([0.2, 0.0, 0.0])
        v_com.shape = (3,1)

        initial_theta = 0
        print(v_com)
        print(lb_foot)
        reference = [(0.1, 0., 0.2)] * 5 + [(0.1, 0., -0.1)] * 10 + [(0.1, 0., 0.)] * 10
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

        footstep_planner = FootstepPlanner(
            v_com,
            lb_foot,
            lf_foot,
            rb_foot,
            rf_foot,
            initial_theta,
            params
            )

        for step in footstep_planner.plan:
            x_lb_foot = step['pos']["lb"][0]
            y_lb_foot = step['pos']["lb"][1]

            x_rb_foot = step['pos']["rb"][0]
            y_rb_foot = step['pos']["rb"][1]

            x_lf_foot = step['pos']["lf"][0]
            y_lf_foot = step['pos']["lf"][1]

            x_rf_foot = step['pos']["rf"][0]
            y_rf_foot = step['pos']["rf"][1]

            display_marker(ground, 'ground_link', position_in_world_coords=[x_lb_foot,y_lb_foot,0.5],
                    color= [255, 0, 0], print_bodieds_of_the_object=False)
            display_marker(ground, 'ground_link', position_in_world_coords=[x_rb_foot,y_rb_foot,0.5],
                    color= [0, 0, 255], print_bodieds_of_the_object=False)
            display_marker(ground, 'ground_link', position_in_world_coords=[x_lf_foot,y_lf_foot,0.5],
                    color= [0, 255, 0], print_bodieds_of_the_object=False)
            display_marker(ground, 'ground_link', position_in_world_coords=[x_rf_foot,y_rf_foot,0.5],
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
