import numpy as np
import casadi as cs
from casadi import SX, MX, cos, sin, vertcat, horzcat
#from math import sin,cos

def compute_skew(vector):
  v1 = vector[0]
  v2 = vector[1]
  v3 = vector[2]

  matrix = MX.zeros(3, 3)
  matrix[0, 1] = -v3
  matrix[1, 0] = v3
  matrix[0, 2] = v2
  matrix[2, 0] = -v2
  matrix[1, 2] = -v1
  matrix[2, 1] = v1 

  return matrix

class mpc:
  def __init__(self, initial, footstep_planner, params):
    # parameters
    #self.params = params
    self.N = 200 #params['N']
    self.delta = 0.1 #params['world_time_step']
    #self.h = params['h']
    #self.eta = params['eta'] #non serve
    #self.foot_size = params['foot_size']
    #self.initial = initial
    #self.footstep_planner = footstep_planner
    #self.sigma = lambda t, t0, t1: np.clip((t - t0) / (t1 - t0), 0, 1) # piecewise linear sigmoidal function

    # optimization problem
    self.opt = cs.Opti('conic')
    p_opts = {"expand": True}
    s_opts = {"max_iter": 1000, "verbose": False}
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

    self.m = 12.72

    I_body = MX.zeros(3,3)
    I_body[0,0] = 0.24
    I_body[1,1] = 1
    I_body[2,2] = 1

    # TODO: totti-check
    I_hat = Rz @ I_body @ Rz.T 
    I_hat_inv = cs.inv(I_hat)

    r1 = self.opt.parameter(3)
    r2 = self.opt.parameter(3)
    r3 = self.opt.parameter(3)
    r4 = self.opt.parameter(3)

    r1_skew = compute_skew(r1)
    r2_skew = compute_skew(r2)
    r3_skew = compute_skew(r3)
    r4_skew = compute_skew(r4)


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

    # Dynamic model: A
    B_row1 = horzcat(Z3, Z3, Z3, Z3)
    B_row2 = horzcat(Z3, Z3, Z3, Z3)
    B_row3 = horzcat(I_hat_inv@r1_skew, I_hat_inv@r2_skew, I_hat_inv@r3_skew, I_hat_inv@r4_skew)
    B_row4 = horzcat(I3 /self.m, I3 /self.m, I3 /self.m, I3 /self.m)
    B_row5 = MX.zeros(1, 12)
    self.B = vertcat(B_row1, B_row2, B_row3, B_row4, B_row5)


    ## TODO: dynamics
    self.f = lambda x, u: self.A @ x + self.B @ u
  
    # State constraint (19)
    for i in range(self.N):
      self.opt.subject_to(self.X[:, i + 1] == self.X[:, i] + self.delta * self.f(self.X[:, i], self.U[:, i]))

    # Cost function
    x_des = self.opt.parameter(13, self.N+1)
    cost = cs.sumsqr(self.U) + \
           100 * cs.sumsqr(self.X[0:3, :] - x_des[0:3, :]) + \
           100 * cs.sumsqr(self.X[3:6, :] - x_des[3:6, :]) + \
           100 * cs.sumsqr(self.X[6:9, :] - x_des[6:9, :]) + \
           100 * cs.sumsqr(self.X[9:12, :] - x_des[9:12, :]) 

    self.opt.minimize(cost)

    # initial state constraint
    self.opt.subject_to(self.X[:, 0] == self.x0_param)

    # Force equality constraint (21)
    swing_param = self.opt.parameter(4, self.N) # inverti binario array gait
    for i in range(self.N):
      self.opt.subject_to( swing_param[0,i] * self.U[0:3, i] == 0) 
      self.opt.subject_to( swing_param[1,i] * self.U[3:6, i] == 0)
      self.opt.subject_to( swing_param[2,i] * self.U[6:9, i] == 0)
      self.opt.subject_to( swing_param[3,i] * self.U[9:12, i] == 0) 

    # Force inequality constraint (20)
    # n_leg = 4
    # (22) f_minmax: 2*n_leg = 8
    # (23) fx: 2*2*n_leg
    # (24) fx: 2*2*n_leg
    # TODO: check parameters
    f_min = 10
    f_max = 666
    mu = 0.4 
    for i in range(self.N):
      # (22)
      for j in range(2, 12, 3):
        self.opt.subject_to( f_min <= self.U[j, i] )
        self.opt.subject_to( self.U[j, i] >= f_max )

      # (23)
      for j in range(1, 12 , 3):
        self.opt.subject_to( -mu*self.U[j+1,i] <= self.U[j, i] )
        self.opt.subject_to( self.U[j, i] <= mu*self.U[j+1,i]  )

        self.opt.subject_to( -mu*self.U[j+1,i] <= -self.U[j, i] )
        self.opt.subject_to( -self.U[j, i] <= mu*self.U[j+1,i]  )
      
      # (24)
      for j in range(0, 12 , 3):
        self.opt.subject_to( -mu*self.U[j+2,i] <= self.U[j, i] )
        self.opt.subject_to( self.U[j, i] <= mu*self.U[j+2,i]  )

        self.opt.subject_to( -mu*self.U[j+2,i] <= -self.U[j, i] )
        self.opt.subject_to( -self.U[j, i] <= mu*self.U[j+2,i]  )

    # state
    #self.x = np.zeros(9)
    #self.lip_state = {'com': {'pos': np.zeros(3), 'vel': np.zeros(3), 'acc': np.zeros(3)},
    #                  'zmp': {'pos': np.zeros(3), 'vel': np.zeros(3)}}

  def solve(self, current, t):
    self.x = np.array([current['com']['pos'][0], current['com']['vel'][0], current['zmp']['pos'][0],
                       current['com']['pos'][1], current['com']['vel'][1], current['zmp']['pos'][1],
                       current['com']['pos'][2], current['com']['vel'][2], current['zmp']['pos'][2]])
    
    mc_x, mc_y, mc_z = self.generate_moving_constraint(t)

    # solve optimization problem
    self.opt.set_value(self.x0_param, self.x)
    self.opt.set_value(self.zmp_x_mid_param, mc_x)
    self.opt.set_value(self.zmp_y_mid_param, mc_y)
    self.opt.set_value(self.zmp_z_mid_param, mc_z)

    sol = self.opt.solve()
    self.x = sol.value(self.X[:,1])
    self.u = sol.value(self.U[:,0])

    self.opt.set_initial(self.U, sol.value(self.U))
    self.opt.set_initial(self.X, sol.value(self.X))

    ## create output LIP state
    #self.lip_state['com']['pos'] = np.array([self.x[0], self.x[3], self.x[6]])
    #self.lip_state['com']['vel'] = np.array([self.x[1], self.x[4], self.x[7]])
    #self.lip_state['zmp']['pos'] = np.array([self.x[2], self.x[5], self.x[8]])
    #self.lip_state['zmp']['vel'] = self.u
    #self.lip_state['com']['acc'] = self.eta**2 * (self.lip_state['com']['pos'] - self.lip_state['zmp']['pos']) + np.hstack([0, 0, - self.params['g']])

    contact = self.footstep_planner.get_phase_at_time(t)
    if contact == 'ss':
      contact = self.footstep_planner.plan[self.footstep_planner.get_step_index_at_time(t)]['foot_id']

    return self.lip_state, contact
  
  def generate_moving_constraint(self, t):
    mc_x = np.full(self.N, (self.initial['lfoot']['pos'][3] + self.initial['rfoot']['pos'][3]) / 2.)
    mc_y = np.full(self.N, (self.initial['lfoot']['pos'][4] + self.initial['rfoot']['pos'][4]) / 2.)
    time_array = np.array(range(t, t + self.N))
    for j in range(len(self.footstep_planner.plan) - 1):
      fs_start_time = self.footstep_planner.get_start_time(j)
      ds_start_time = fs_start_time + self.footstep_planner.plan[j]['ss_duration']
      fs_end_time = ds_start_time + self.footstep_planner.plan[j]['ds_duration']
      fs_current_pos = self.footstep_planner.plan[j]['pos'] if j > 0 else np.array([mc_x[0], mc_y[0]])
      fs_target_pos = self.footstep_planner.plan[j + 1]['pos']
      mc_x += self.sigma(time_array, ds_start_time, fs_end_time) * (fs_target_pos[0] - fs_current_pos[0])
      mc_y += self.sigma(time_array, ds_start_time, fs_end_time) * (fs_target_pos[1] - fs_current_pos[1])

    return mc_x, mc_y, np.zeros(self.N)

if __name__ == '__main__':
  mpc_try = mpc(0,0,0)