import dartpy as dart
import os
from lite3_controller import Lite3Controller
import numpy as np
import copy
from utils import *
def customPreStep(self):
    return

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
    
    for i in range(num_joints):
        joint = lite3.getJoint(i)
        print(f"- Joint {i}: {joint.getName()} (Tipo: {joint.getType()})")

        # body_node = lite3.getBodyNode(i)
        # print(f"- Body Node {i}: {body_node.getName()}")

# set default inertia
    default_inertia = dart.dynamics.Inertia(1e-8, np.zeros(3), 1e-10 * np.identity(3))
    for body in lite3.getBodyNodes():
        if body.getMass() == 0.0:
            body.setMass(1e-8)
            body.setInertia(default_inertia)

    node = Lite3Controller(world, lite3)

    display_marker(lite3, 'HR_HIP', 
                    position_in_world_coords=[0,0,0],
                    color=[0,0,255])#,   
    display_marker(lite3, 'HR_THIGH', 
                    position_in_world_coords=[0,0,0],
                    color=[0,0,255])#,   
    display_marker(lite3, 'HR_SHANK', 
                    position_in_world_coords=[0,0,0],
                    color=[0,0,255])#,  
    
    display_marker(lite3, 'HR_FOOT', 
                    position_in_world_coords=[0,0,0],
                    color=[255,0,0])#,  
    display_marker(lite3, 'HL_FOOT', 
                    position_in_world_coords=[0,0,0],
                    color=[255,0,0])#,
    display_marker(lite3, 'FR_FOOT', 
                    position_in_world_coords=[0,0,0],
                    color=[255,0,0])#,
    display_marker(lite3, 'FL_FOOT', 
                    position_in_world_coords=[0,0,0],
                    color=[255,0,0])#,

    # create world node and add it to viewer
    viewer = dart.gui.osg.Viewer()
    node.setTargetRealTimeFactor(1) # speed up the visualization by 10x
    viewer.addWorldNode(node)

    viewer.setUpViewInWindow(0, 0, 1270, 720)
    viewer.setCameraHomePosition([5., -1., 1.5],
                                 [1.,  0., 0.5],
                                 [0.,  0., 1. ])

    
   
    viewer.run()
