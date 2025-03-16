import numpy as np
import dartpy as dart
import copy
import os

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
            'world_time_step': world.getTimeStep(),
            'first_swing': 'rfoot',
            'µ': 0.5,
            'N': 100,
            'dof': self.lite3.getNumDofs(),
        }
        self.params['eta'] = np.sqrt(self.params['g'] / self.params['h'])


        self.lf_sole = lite3.getBodyNode('FL_FOOT')
        self.rf_sole = lite3.getBodyNode('FR_FOOT')
        self.lb_sole = lite3.getBodyNode('HL_FOOT')
        self.rb_sole = lite3.getBodyNode('HR_FOOT')
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
        

        lf_sole_pos = self.lf_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
        rf_sole_pos = self.rf_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
        lb_sole_pos = self.lb_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
        rb_sole_pos = self.rb_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
        #feet_center = (lf_sole_pos +  rf_sole_pos + lb_sole_pos + rb_sole_pos)/4. 
        # Ochio perche a seconda di come setti initial_config 
        # # li ho printati nella config tutto 0 e usiamo quelli fissati per ogni inital_config
        # self.lite3.setPosition(2, -3.14/4)
        # self.lite3.setPosition(3, -0.11352459)
        # self.lite3.setPosition(4, -0.09843932)
        self.lite3.setPosition(5, 0.5)


