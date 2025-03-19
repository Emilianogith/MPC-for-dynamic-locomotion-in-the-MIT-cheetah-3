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
        

        fl_sole_pos = self.fl_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
        fr_sole_pos = self.fr_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
        hl_sole_pos = self.hl_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
        hr_sole_pos = self.hr_sole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).translation()
        #feet_center = (lf_sole_pos +  rf_sole_pos + lb_sole_pos + rb_sole_pos)/4. 
        # Ochio perche a seconda di come setti initial_config 
        # # li ho printati nella config tutto 0 e usiamo quelli fissati per ogni inital_config
        # self.lite3.setPosition(2, -3.14/4)
        # self.lite3.setPosition(3, -0.11352459)
        # self.lite3.setPosition(4, -0.09843932)
        self.lite3.setPosition(5, 0.5)


    def CustomPreStep(self):
        J = {'FL_FOOT' : self.lite3.getLinearJacobian(self.fl_sole,         inCoordinatesOf=dart.dynamics.Frame.World())[:,6:9],
             'FR_FOOT' : self.lite3.getLinearJacobian(self.fr_sole,         inCoordinatesOf=dart.dynamics.Frame.World())[:,9:12],
             'HL_FOOT'   : self.lite3.getLinearJacobian(self.hl_sole,       inCoordinatesOf=dart.dynamics.Frame.World())[:,12:15],
             'HR_FOOT' : self.lite3.getLinearJacobian(self.hr_sole,        inCoordinatesOf=dart.dynamics.Frame.World())[:,15:],
             }
        print(J)

        #Test value for controls
        f = {'FL_FOOT' : [0, 0, 30],
             'FR_FOOT' : [0, 0, 30],
             'HL_FOOT' : [0, 0, 30],
             'HR_FOOT' : [0, 0, 30],
            }
        print('cc')
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
        #actual commands
        i = 0
        print(tau)
        for task in joint_name:
            if i in [1,2,3,5,6,7,9,10,11,13,14,15]:
                self.lite3.setCommand(self.lite3.getDof(i).getIndexInSkeleton(), tau[task][0]) #non sò se funziona come scrittura per ottenere i valori dell'array
                self.lite3.setCommand(self.lite3.getDof(i).getIndexInSkeleton(), tau[task][1])
                self.lite3.setCommand(self.lite3.getDof(i).getIndexInSkeleton(), tau[task][2])
            i+=1
        self.time += 1    


