# MPC for dynamic locomotion of Lite 3

## 📌 Abstract

This work presents the development of a simulated quadruped locomotion system controlled through
Model Predictive Control. To formulate the problem as a convex Quadratic Program suitable for MPC,
several simplifying assumptions were introduced to approximate the robot’s dynamics.
Simulations were performed using the Lite3 quadruped robot from DeepRobotics. A dedicated footstep
planner, along with ground and swing leg controllers, was implemented and kept as simple as possible
to reduce the overall complexity of the work while maintaining effective locomotion behavior.
The simulation results demonstrate that the MPC framework can successfully control quadruped walking.
Various gaits were tested under different conditions, showing that the controller is robust and capable of
achieving stable and accurate locomotion.

| Trotting | Pronking |
|----------|----------|
| ![Trotting](presentation/trotting.gif) | ![Pronking](presentation/pronking.gif) |

| Ambling | Galopping |
|---------|-----------|
| ![Ambling](presentation/ambling.gif) | ![Galopping](presentation/galopping.gif) |

---

## 📁 Repository Structure

```
.
├── lite3_urdf/                   # URDF model of the Lite3 robot
├── presentation/                 
   ├── UR_presentation            # Presentation of the project
   ├── UR_report.pdf              # Detailed report of the project
   ├── UR_paper.pdf               # Reference paper
├── src/
   ├── main.py                       # Runnable: Main entry point for simulation
   ├── plot.py                       # Runnable: Plot some data decided into the script
   ├── foot_trajectory_generator.py  # Swing trajectory generation
   ├── footstep_planner.py           # Main footstep planner script
   ├── mpc.py                        # MPC optimization logic
   ├── utils.py                      # Utility functions
   ├── logger.py                     # Logging utilities
├── README.md                     # Project documentation
```

---

## 🧠 Modules Overview

### 🦶 Footstep Planner

The footstep planner is based on a virtual unicycle model located under the robot's center of mass (CoM). Foot placements are computed by integrating a constant reference velocity $v_{\text{ref}}$ and angular velocity $\omega_{\text{ref}}$ over time.

The unicycle motion evolves as:

$$
p_{\text{CoM}}(t+\Delta t) = p_{\text{CoM}}(t) + R(\theta) \cdot v_{\text{ref}} \cdot \Delta t
$$

$$
\theta(t+\Delta t) = \theta(t) + \omega_{\text{ref}} \cdot \Delta t
$$

This strategy allows generating different gait patterns by defining alternating support and swing phases for the legs.

---

### 📈 Trajectory Generator

Swing trajectories are generated using polynomial interpolation:

- **Cubic interpolation** in the horizontal plane:

$$
p(t) = p_i + (p_f - p_i)\left(-2\left(\frac{t}{T}\right)^3 + 3\left(\frac{t}{T}\right)^2\right), \quad t \in [0, T]
$$

- **Quartic interpolation** for the vertical (z) axis:

$$
z(t) = at^4 + bt^3 + ct^2, \quad t \in [0, T]
$$

with coefficients:

$$
a = \frac{16h}{T^4}, \quad b = -\frac{32h}{T^3}, \quad c = \frac{16h}{T^2}
$$

These trajectories ensure smooth lift-off and landing with zero velocity and acceleration at boundaries.

---

### 🔄 Model Predictive Controller (MPC)

The MPC computes optimal ground reaction forces over a finite time horizon using a simplified rigid-body model. It enforces dynamic feasibility, swing/stance contact constraints, and friction cone conditions.

---

### 🦿 Controllers

#### Ground Controller

The ground controller transforms the optimal ground reaction force $f_i$ for each stance leg into joint torques:

$$
\tau_i = J_i^\top R_i^\top f_i
$$

Where:
- $J_i$ is the foot Jacobian,
- $R_i$ is the rotation from robot to world frame,
- $\tau_i$ is the joint torque vector.

#### Swing Controller

The swing controller combines feedback and feedforward terms to track the foot trajectory:

$$
\tau_i = J_i^\top \Big(  K_p \cdot (p_{i,\text{ref}} - p_i) + K_d \cdot (v_{i,\text{ref}} - v_i) \Big) + \tau_{i,\text{ff}}
$$

Where the feedforward torque is:

$$
\tau_{i,\text{ff}} = J_i^\top M_i \left( a_{i,\text{ref}} - \dot{J}_i \dot{q}_i \right) + C_i \dot{q}_i + G_i
$$

This ensures smooth and accurate motion tracking during the swing phase.


---


## 🛠️ Other

- Occasional ground penetration, problem arise in simulation side
- Visual meshes for the robot had to be manually rotated to ensure proper visualization in simulation.
- Required libraries are: numpy, matplotlib, osqp, dartpy
- Runnable scripts are: main.py, plot.py
- Press space bar to start the simulation

---

## 📚 References

- [Di Carlo et al. (2018). *Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control*. IEEE/RSJ IROS, Madrid, Spain.](https://ieeexplore.ieee.org/document/8594448)
- [Stark et al. (2025). *Benchmarking Different QP Formulations and Solvers for Dynamic Quadrupedal Walking*. arXiv:2502.01329v1 [cs.RO]](https://arxiv.org/abs/2502.01329)
- [Zhu (GitHub). *Lite 3 URDF Configuration*. TopHillRobotics Repository](https://github.com/TopHillRobotics/quadruped-robot/blob/mpc-wbc/quadruped/config/lite3/lite3_robot.yaml)
- [Reference for implementation: DIAG-Robotics-Lab/ismpc](https://github.com/DIAG-Robotics-Lab/ismpc)

---

## 👥 Authors

Vittorio Pisapia([VittorioPisapia](https://github.com/VittorioPisapia)), Jacopo Tedeschi([jacopotdsc](https://github.com/jacopotdsc)), Emiliano Paradiso([Emilianogith](https://github.com/Emilianogith)), Brian Piccione([HatoKng](https://github.com/HatoKng))
