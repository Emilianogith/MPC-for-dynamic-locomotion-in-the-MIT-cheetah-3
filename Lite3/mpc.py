import numpy as np
import casadi as cs
from casadi import SX, MX, cos, sin, vertcat, horzcat
from utils import *
#from math import sin,cos
from foot_trajectory_generator import FootTrajectoryGenerator

class MPC:
  def __init__(self, lite3, initial, footstep_planner, params):
    # parameters
    self.params = params
    self.lite3 = lite3
    self.N = params['N']
    self.delta = params['world_time_step']
    self.h = params['h']
    self.foot_size = params['foot_size']
    self.mu = params['µ']
    self.initial = initial
    self.footstep_planner = footstep_planner
    #self.sigma = lambda t, t0, t1: np.clip((t - t0) / (t1 - t0), 0, 1) # piecewise linear sigmoidal function
    self.com_pos_start = initial['com_position']
    self.yaw_start = initial['yaw']

    self.trajectory_generator = FootTrajectoryGenerator(
            footstep_planner = self.footstep_planner,
            params = self.params
        )

    # optimization problem
    self.opt = cs.Opti('conic')
    p_opts = {"expand": True}
    s_opts = {"max_iter": 1000, 
              "verbose": False,
              #"error_on_fail": True
              }
    self.opt.solver("osqp", p_opts, s_opts)

    self.U = self.opt.variable(12, self.N)
    self.X = self.opt.variable(13, self.N + 1)

    self.x0_param = self.opt.parameter(13)   #theta (rpy), p, omega, pdot, g

    # Inertia and mass parameters
    yaw = self.x0_param[2] #self.opt.parameter(1) #MX.sym("yaw", 1)
    Rz = vertcat(
        horzcat(cos(yaw), -sin(yaw), 0),
        horzcat(sin(yaw),  cos(yaw), 0),
        horzcat(0,            0,           1)
    )

    self.m = 9.72 #12.72

    I_body_inv = MX.zeros(3,3)
    I_body_inv[0,0] = 1/0.24
    I_body_inv[1,1] = 1/1
    I_body_inv[2,2] = 1/1

    # TODO: totti-check
    #I_hat = Rz @ I_body @ Rz.T 
    I_hat_inv = Rz @I_body_inv @ Rz.T  #cs.inv(I_hat)

    #self.r1 = self.opt.parameter(3)
    #self.r2 = self.opt.parameter(3)
    #self.r3 = self.opt.parameter(3)
    #self.r4 = self.opt.parameter(3)

    self.r1_skew = self.opt.parameter(3,self.N*3) #compute_skew(self.r1)
    self.r2_skew = self.opt.parameter(3,self.N*3) #compute_skew(self.r2)
    self.r3_skew = self.opt.parameter(3,self.N*3) #compute_skew(self.r3)
    self.r4_skew = self.opt.parameter(3,self.N*3) #compute_skew(self.r4)


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


    ## TODO: dynamics 
    self.f = lambda x, u, i: self.A @ x + self.B[:,12*i:12*(1+i)] @ u
  
    # State constraint (19)
    for i in range(self.N):
      self.opt.subject_to(self.X[:, i + 1] == self.X[:, i] + self.delta * self.f(self.X[:, i], self.U[:, i],i))
      
    # Cost function
    self.x_des = self.opt.parameter(13, self.N+1)
    cost = 0.0 * cs.sumsqr(self.U) + \
           1 * cs.sumsqr(self.X[0:3,  :] - self.x_des[0:3, :]) + \
           100 * cs.sumsqr(self.X[3:5,  :] - self.x_des[3:5, :]) + \
           50 * cs.sumsqr(self.X[5,  :] - self.x_des[5, :]) + \
           1 * cs.sumsqr(self.X[6:9,  :] - self.x_des[6:9, :]) + \
           1 * cs.sumsqr(self.X[9:12, :] - self.x_des[9:12, :]) + \
           0 * cs.sumsqr(self.X[12, :] - self.x_des[12, :])
          #(0 10 500 500 10 10 0)
    self.opt.minimize(cost)

    # initial state constraint
    self.opt.subject_to(self.X[:, 0] == self.x0_param)

    # Force equality constraint (21)
    self.swing_param = self.opt.parameter(4, self.N) # inverti binario array gait
    for i in range(self.N):
      self.opt.subject_to( self.swing_param[0,i] * self.U[0:3, i] == 0) 
      self.opt.subject_to( self.swing_param[1,i] * self.U[3:6, i] == 0)
      self.opt.subject_to( self.swing_param[2,i] * self.U[6:9, i] == 0)
      self.opt.subject_to( self.swing_param[3,i] * self.U[9:12, i] == 0) 


    # Force inequality constraint (20)
    # TODO: check parameters
    f_min = -10   # mi funzionava con -10
    f_max = 1000    # fino a tipo 700 funziona
    mu = 0.5 
    for i in range(self.N):

      self.opt.subject_to( self.X[12,i] == self.params['g']) 

      # TODO
      # (22)                                                   # Paradossalmente il robot va meglio senza questo 
      for j in range(2, 12, 3):                                # constraint. Mettendo valori di f_min MAGGIORI di 0 la simulazione runna,
        self.opt.subject_to( f_min <= self.U[j, i] )           # ma trova soluzioni per le gambe di support o nulle o comunque molto basse,
        self.opt.subject_to( self.U[j, i] <= f_max )           # e quindi vanno dove gli pare

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


  def solve(self, t):

    gait = self.footstep_planner.get_phase_at_time(t)
    # Durante il double support dovremmo mettere una velocità nulla o molto bassa (frazione della v di riferimento),
    # sennò il doggo continua a portare il busto in avanti, sbilanciandosi
    if np.array_equal(gait, np.array([1,1,1,1])):
      v_com_gait = self.params['v_com_ref']*0
    else:
      v_com_gait = self.params['v_com_ref']
    
    #---------------------- Retreive state ----------------------  
    # ( rpy - CoM, Angular Velocity, Cartesian Velocity, gravity )
    current_state = self.lite3.retrieve_state() # TODO: totti check di stampe
    state_rpy = np.array([current_state['TORSO']['pos']]).T
    state_com = np.array([current_state['com']['pos']]).T
    state_av  = np.array([current_state['TORSO']['vel']]).T
    state_lv  = np.array([current_state['com']['vel']]).T
    state_g   = self.params['g']

    self.x = np.vstack([state_rpy, state_com, state_av, state_lv, state_g])


    #---------------------- x_des definition ----------------------
    # x_des: ( rpy - CoM, Angular Velocity, Cartesian Velocity, gravity )
    # setting constant values
    x_des_num = np.zeros((13, self.N+1))
    x_des_num[:2, :] = np.ones((2, self.N+1)) * [ [self.initial['roll']], [self.initial['pitch']]] # roll, pitch state
    x_des_num[8, :]  = np.ones((1, self.N+1)) * self.params['theta_dot'] # Constant velocity for yaw
    x_des_num[9:12, :] = np.ones((3, self.N+1)) * v_com_gait.reshape(3,1) # Constant velocity of CoM
    x_des_num[12, :]   = np.ones((1, self.N+1)) * self.params['g'] # Gravity term
    x_des_num[2,0] = self.yaw_start + self.params['theta_dot']*self.delta
    x_des_num[3:6,0] = self.com_pos_start + v_com_gait*self.delta
    x_des_num[5,0] = self.h

    for i in range(1, self.N+1):
      x_des_num[2, i] = x_des_num[2, i-1] + self.params['theta_dot']*self.delta   # Integrating yaw
      x_des_num[3:6, i] = x_des_num[3:6, i-1] + v_com_gait*self.delta # Integrating com_pos

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
    self.opt.set_value(self.r1_skew, r1_skew_num ) # Substitution for matrix B 
    self.opt.set_value(self.r2_skew, r2_skew_num ) # Substitution for matrix B
    self.opt.set_value(self.r3_skew, r3_skew_num ) # Substitution for matrix B
    self.opt.set_value(self.r4_skew, r4_skew_num ) # Substitution for matrix B
    
    # Equality constraint (21): 
    swing_inverted = np.zeros((4, self.N))
    for i in range(self.N): 
      swing_value = self.footstep_planner.get_phase_at_time(t+i)
      swing_inverted[:, i] = np.array([1, 1, 1, 1]) - np.array(swing_value)

    self.opt.set_value(self.swing_param, swing_inverted)
    
    
    #print("------------------------------------------------------------------------------------------------")

    self.opt.set_value(self.x_des, x_des_num) # Substitution desired state
    

    sol = self.opt.solve()

    #aggiornare variabile integrazione
    self.com_pos_start = x_des_num[3:6,0]
    self.yaw_start = x_des_num[2,0]

    self.x = sol.value(self.X[:,1])
    self.x_plot = sol.value(self.X[3:6,:])
    self.u = sol.value(self.U[:,0]) #forces
    self.u_plot = sol.value(self.U[:,:]) #forces

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

    forces_z = {
      'FL_FOOT' : self.u[2],
      'FR_FOOT' : self.u[5],
      'HL_FOOT' : self.u[8],
      'HR_FOOT' : self.u[11],
    }
    #if t % 10 == 0 or t == 0:
    #  log_mpc(self, t, x_des_num, swing_inverted, forces)
  
    #if t == 100:
    #  plot_com_and_forces(self.N , self.x_plot[:,:self.N], x_des_num[3:6,:self.N], forces_plot, t)
    #if t == 100:
    #  plot_com_and_forces(self.N , self.x_plot[:,:self.N], x_des_num[3:6,:self.N], forces_plot)
    #if t == 150:
    #  plot_com_and_forces(self.N , self.x_plot[:,:self.N], x_des_num[3:6,:self.N], forces_plot)
    #if t == 200:
    #  plot_com_and_forces(self.N , self.x_plot[:,:self.N], x_des_num[3:6,:self.N], forces_plot)

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