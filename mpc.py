import numpy as np
import casadi as cs
from casadi import SX, MX, cos, sin, vertcat, horzcat
from utils import *
#from math import sin,cos
from foot_trajectory_generator import FootTrajectoryGenerator

class MPC:
  """This class implements a whole-body MPC controller that computes optimal ground reaction
    forces over a prediction horizon to track desired center of mass motion and torso orientation.
    The controller integrates dynamics, contact constraints, and footstep planning to maintain balance
    and produce dynamic walking.

    Main Components:
    - Dynamics model: linearized rigid-body dynamics with contact wrenches.
    - Constraints: force equality (zero during swing), force bounds, and friction cone.
    - Cost function: penalizes deviation from desired motion and control effort.
    - Integration with footstep planner and swing foot trajectory generator.

    Methods:
    - solve(t, logger): solves the MPC problem at time `t` and returns force commands.
    - update_r_num(time, leg_name, next_com): computes relative foot positions for dynamics.
  """
  
  def __init__(self, lite3, initial, footstep_planner, params):
   
    # parameters
    self.params = params
    self.lite3 = lite3
    self.N = params['N']
    self.delta = params['world_time_step']
    self.h = params['h']
    self.mu = params['µ']
    self.initial = initial
    self.footstep_planner = footstep_planner
    self.com_pos_start = initial['com_position']
    self.com_pos_start[2] = self.h
    self.yaw_start = initial['yaw']

    self.trajectory_generator = FootTrajectoryGenerator(
            footstep_planner = self.footstep_planner,
            params = self.params
        )

    f_min = 3
    f_max = 100

    # optimization problem
    self.opt = cs.Opti('conic')
    p_opts = {"expand": True}
    s_opts = {"max_iter": 1000, 
              "verbose": False,
              #"error_on_fail": True
              }
    self.opt.solver("osqp", p_opts, s_opts)

    # Defining the variables
    self.U = self.opt.variable(12, self.N)
    self.X = self.opt.variable(13, self.N + 1)

    self.x0_param = self.opt.parameter(13)   #theta (rpy), p, omega, pdot, g

    # Inertia and mass parameters
    yaw = self.x0_param[2] 
    Rz = vertcat(
        horzcat(cos(yaw), -sin(yaw), 0),
        horzcat(sin(yaw),  cos(yaw), 0),
        horzcat(0,            0,           1)
    )

    self.m = 8.885 #8.782 # NON TOCCARE

    I_body_inv = MX.zeros(3,3)
    I_body_inv[0,0] = 1/0.24
    I_body_inv[1,1] = 1/1
    I_body_inv[2,2] = 1/1

    I_hat_inv = Rz @I_body_inv @ Rz.T  

    self.r1_skew = self.opt.parameter(3,self.N*3) 
    self.r2_skew = self.opt.parameter(3,self.N*3) 
    self.r3_skew = self.opt.parameter(3,self.N*3) 
    self.r4_skew = self.opt.parameter(3,self.N*3) 


    # Dynamic model: A
    Z3 = MX.zeros(3, 3)
    Z4 = MX.zeros(3, 1)
    I3 = MX.eye(3)
    A_row1 = horzcat(Z3, Z3, Rz, Z3, Z4)
    A_row2 = horzcat(Z3, Z3, Z3, I3, Z4)
    A_row3 = MX.zeros(3, 13)
    A_row4 = MX.zeros(3, 13)
    A_row4[2, 12] = 1
    A_row5 = MX.zeros(1, 13)
    self.A = vertcat(A_row1, A_row2, A_row3, A_row4, A_row5)

    # Dynamic model: B
    self.B = MX.zeros(13,self.N*12)
    for i in range(self.N):
      B_row1 = horzcat(Z3, Z3, Z3, Z3)
      B_row2 = horzcat(Z3, Z3, Z3, Z3)
      B_row3 = horzcat(I_hat_inv@self.r1_skew[:,3*i:3*(1+i)], I_hat_inv@self.r2_skew[:,3*i:3*(1+i)], I_hat_inv@self.r3_skew[:,3*i:3*(1+i)], I_hat_inv@self.r4_skew[:,3*i:3*(1+i)])
      B_row4 = horzcat(I3 /self.m, I3 /self.m, I3 /self.m, I3 /self.m)
      B_row5 = MX.zeros(1, 12)
      B_0 = vertcat(B_row1, B_row2, B_row3, B_row4, B_row5)
      self.B[:,12*i:12*(1+i)] = B_0

    # Dyanamics
    self.f = lambda x, u, i: self.A @ x + self.B[:,12*i:12*(1+i)] @ u

    # initial state constraint
    self.opt.subject_to(self.X[:, 0] == self.x0_param)
  
    # State constraint (19)
    for i in range(self.N):
      self.opt.subject_to(self.X[:, i + 1] == self.X[:, i] + self.delta * self.f(self.X[:, i], self.U[:, i],i))

    # Cost function
    self.x_des = self.opt.parameter(13, self.N+1)
    cost = 0.0 * cs.sumsqr(self.U) + \
           10000 * cs.sumsqr(self.X[0,  :] - self.x_des[0, :])  + \
           27000 * cs.sumsqr(self.X[1,  :] - self.x_des[1, :])  + \
           10000 * cs.sumsqr(self.X[2,  :] - self.x_des[2, :])  + \
           270000 * cs.sumsqr(self.X[3,  :] - self.x_des[3, :])  + \
           270000 * cs.sumsqr(self.X[4,  :] - self.x_des[4, :])  + \
           270000 * cs.sumsqr(self.X[5,  :] - self.x_des[5, :])  + \
           10000 * cs.sumsqr(self.X[6,  :] - self.x_des[6, :])  + \
           10000 * cs.sumsqr(self.X[7 , :] - self.x_des[7, :])  + \
           10000 * cs.sumsqr(self.X[8, :]  - self.x_des[8, :])  + \
           16000 * cs.sumsqr(self.X[9, :]  - self.x_des[9, :])  + \
           16000 * cs.sumsqr(self.X[10, :] - self.x_des[10, :]) + \
           16000 * cs.sumsqr(self.X[11, :] - self.x_des[11, :]) + \
           0.0 * cs.sumsqr(self.X[12, :] - self.x_des[12, :]) 
    
    self.opt.minimize(cost)

    # Force equality constraint (21)
    self.swing_param = self.opt.parameter(4, self.N) 
    for i in range(self.N):
      self.opt.subject_to( self.swing_param[0,i] * self.U[0:3, i] == 0) 
      self.opt.subject_to( self.swing_param[1,i] * self.U[3:6, i] == 0)
      self.opt.subject_to( self.swing_param[2,i] * self.U[6:9, i] == 0)
      self.opt.subject_to( self.swing_param[3,i] * self.U[9:12, i] == 0) 


    # Force inequality constraint (20)
    for i in range(self.N):
      self.opt.subject_to( self.X[12,i] == self.params['g']) 

      # (22)
      index = 0                                                   
      for j in range(2, 12, 3):
        cond = 1-self.swing_param[index,i]                         
        self.opt.subject_to( cond*f_min <= cond*self.U[j, i] )           
        self.opt.subject_to( cond*self.U[j, i] <= cond*f_max )
        index +=1     

      # (24)
      for j in range(1, 12 , 3):
        self.opt.subject_to( -self.mu*self.U[j+1,i] <= self.U[j, i] )
        self.opt.subject_to( self.U[j, i] <= self.mu*self.U[j+1,i]  )

        self.opt.subject_to( -self.mu*self.U[j+1,i] <= -self.U[j, i] )
        self.opt.subject_to( -self.U[j, i] <= self.mu*self.U[j+1,i]  )

      # (23)
      for j in range(0, 12 , 3):
        self.opt.subject_to( -self.mu*self.U[j+2,i] <= self.U[j, i] )
        self.opt.subject_to( self.U[j, i] <= self.mu*self.U[j+2,i]  )

        self.opt.subject_to( -self.mu*self.U[j+2,i] <= -self.U[j, i] )
        self.opt.subject_to( -self.U[j, i] <= self.mu*self.U[j+2,i]  )


  def solve(self, t, logger):
       
    v_com_gait = self.params['v_com_ref']
    omega = self.params['theta_dot']

    if self.footstep_planner.get_step_index_at_time(t) == self.params['total_steps'] - 1:
      v_com_gait = self.params['v_com_ref']*0
      omega = self.params['theta_dot']*0
  
    


    #---------------------- Retreive state ----------------------  
    # ( rpy - CoM, Angular Velocity, Cartesian Velocity, gravity )
    current_state = self.lite3.retrieve_state() 
    state_rpy = np.array([current_state['TORSO']['pos']]).T
    state_com = np.array([current_state['com']['pos']]).T

    state_av  = np.array([current_state['TORSO']['vel']]).T
    state_lv  = np.array([current_state['com']['vel']]).T
    state_g   = self.params['g']

    self.x = np.vstack([state_rpy, state_com, state_av, state_lv, state_g])

    #---------------------- x_des definition ----------------------
    # x_des: ( rpy - CoM, Angular Velocity, Cartesian Velocity, gravity )
    x_des_num = np.zeros((13, self.N+1))
    x_des_num[:2, :] = np.ones((2, self.N+1)) * [ [self.initial['roll']], [self.initial['pitch']]]       # roll, pitch state
    x_des_num[8, :]  = np.ones((1, self.N+1)) * omega                                                    # Constant velocity for yaw
    x_des_num[9:12, :] = 1*np.ones((3, self.N+1)) * v_com_gait.reshape(3,1)                              # Constant velocity of CoM
    x_des_num[12, :]   = np.ones((1, self.N+1)) * self.params['g']                                       # Gravity term
    x_des_num[2,0] = self.yaw_start 
    x_des_num[3:6,0] = self.com_pos_start 
    


    for i in range(1, self.N+1):
      x_des_num[2, i] = x_des_num[2, i-1] + omega*self.delta                                             # Integrating yaw
      x_des_num[3:6, i] = x_des_num[3:6, i-1] + v_com_gait*self.delta                                    # Integrating com_pos


    #---------------------- Parameter substitutions ----------------------
    r1_skew_num = np.zeros((3,self.N*3))
    r2_skew_num = np.zeros((3,self.N*3))
    r3_skew_num = np.zeros((3,self.N*3))
    r4_skew_num = np.zeros((3,self.N*3))

    r1_num = current_state['FL_FOOT']['pos'][3:] - current_state['com']['pos']
    r2_num = current_state['FR_FOOT']['pos'][3:] - current_state['com']['pos']
    r3_num = current_state['HL_FOOT']['pos'][3:] - current_state['com']['pos']
    r4_num = current_state['HR_FOOT']['pos'][3:] - current_state['com']['pos']

    for i in range(self.N):
        
      r1_skew_num[:,3*i:3*(1+i)] = compute_skew(r1_num)
      r2_skew_num[:,3*i:3*(1+i)] = compute_skew(r2_num)
      r3_skew_num[:,3*i:3*(1+i)] = compute_skew(r3_num)
      r4_skew_num[:,3*i:3*(1+i)] = compute_skew(r4_num)

      future_t = t+i+1
      r1_num = self.update_r_num(future_t,'FL_FOOT',x_des_num[3:6,i+1])
      r2_num = self.update_r_num(future_t,'FR_FOOT',x_des_num[3:6,i+1])
      r3_num = self.update_r_num(future_t,'HL_FOOT',x_des_num[3:6,i+1])
      r4_num = self.update_r_num(future_t,'HR_FOOT',x_des_num[3:6,i+1])


    self.opt.set_value(self.x0_param, self.x)      # Substitution initial state
    self.opt.set_value(self.r1_skew, r1_skew_num ) 
    self.opt.set_value(self.r2_skew, r2_skew_num )
    self.opt.set_value(self.r3_skew, r3_skew_num )
    self.opt.set_value(self.r4_skew, r4_skew_num ) 
    
    # Equality constraint (21): 
    swing_inverted = np.zeros((4, self.N))
    for i in range(self.N): 
      swing_value = self.footstep_planner.get_phase_at_time(t+i)    
      swing_inverted[:, i] = np.array([1, 1, 1, 1]) - np.array(swing_value)

    self.opt.set_value(self.swing_param, swing_inverted)
    self.opt.set_value(self.x_des, x_des_num)    # Substitution desired state
    

    sol = self.opt.solve()

    # integrate refernce trajectory over time based on constant reference values
    self.com_pos_start += v_com_gait*self.delta
    self.yaw_start +=  omega*self.delta

    
    self.x_log = sol.value(self.X[:-1,:])
    self.x_plot = sol.value(self.X[3:6,:])
    self.u = sol.value(self.U[:,0])               #forces
    self.u_plot = sol.value(self.U[:,:])

    self.opt.set_initial(self.U, sol.value(self.U))
    self.opt.set_initial(self.X, sol.value(self.X))

    forces = {
      'FL_FOOT' : self.u[0:3],
      'FR_FOOT' : self.u[3:6],
      'HL_FOOT' : self.u[6:9],
      'HR_FOOT' : self.u[9:12],
    }

    forces_plot = np.array([
                            self.u_plot[2,:],
                            self.u_plot[5,:],
                            self.u_plot[8,:],
                            self.u_plot[11,:],
    ])

    state_rpy_list  = state_rpy.flatten().tolist()
    state_com_list = state_com.flatten().tolist()
    state_av_list  = state_av.flatten().tolist()
    state_lv_list  = state_lv.flatten().tolist()

    x_curr = state_rpy_list + state_com_list + state_av_list + state_lv_list
    
    # log the tracking performance 
    logger.log_tracking_data(x_curr, x_des_num[:-1,0])

    if t == 0:
      #plot_com_and_forces(self.N , self.x_plot[:,:self.N], x_des_num[3:6,:self.N], forces_plot, t)
      logger.log_mpc_predictions(self.x_log, x_des_num[:-1,:], forces_plot, t)
    if t == 80:
      logger.log_mpc_predictions(self.x_log, x_des_num[:-1,:], forces_plot, t)

    return forces
  

  def update_r_num(self, time, leg_name, next_com):
    gait = self.footstep_planner.get_phase_at_time(time)

    if self.footstep_planner.is_swing(leg_name,gait) == 1:
      leg_pos = self.trajectory_generator.generate_feet_trajectories_at_time(time, leg_name)
      leg_pos = leg_pos['pos'][3:]
      r_num = leg_pos - next_com
      return r_num
    else:
      step = self.footstep_planner.get_step_index_at_time(time)
      leg_pos = self.footstep_planner.plan[step]['pos'][leg_name]
      r_num = leg_pos - next_com
      return r_num