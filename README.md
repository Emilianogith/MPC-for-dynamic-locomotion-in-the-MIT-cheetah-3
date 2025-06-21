# MPC for dynamic locomotion of Lite 3

## 📌 Abstract

This project simulates the locomotion of the **Lite3 quadruped robot** using **Model Predictive Control (MPC)** and rigid-body dynamics. The control architecture includes a footstep planner, a swing trajectory generator, and dedicated controllers for both ground and swing legs. The entire system is validated using the **DartPy** simulation framework.

---

## 📁 Repository Structure

```
.
├── lite3_urdf/                   # URDF model of the Lite3 robot
├── paper/                        # Reference materials 
├── foot_trajectory_generator.py  # Swing trajectory generation
├── footstep_planner.py           # Main footstep planner script
├── footstep_planner_backup.py    # Backup version of the planner
├── single_leg_controller.py      # Swing leg controller
├── mpc.py                        # MPC optimization logic
├── main.py                       # Main entry point for simulation
├── utils.py                      # Utility functions
├── logger.py                     # Logging utilities
├── README.md                     # Project documentation
```

---

## 🚀 How to Run

1. **Install dependencies:**
   ```bash
   pip install numpy matplotlib osqp dartpy
   ```

2. **Run MPC simulation in DartPy:**
   ```bash
   python3 sim/main_simulation.py
   ```

press space bar to start the simulation

---

## 🧠 Modules Overview

### 🦶 Footstep Planner

Implements a unicycle-based model to generate a sequence of footsteps using `v_ref` and `omega_ref`. It defines gaits using swing/stay contact patterns and supports trajectory visualization.

### 📈 Trajectory Generator

Interpolates swing foot trajectories using:
- Cubic polynomials (x, y)
- Quartic polynomials (z)

### 🔄 Model Predictive Controller (MPC)

Solves a convex QP problem at every time step:
- Inputs: desired velocity/pose over horizon
- Outputs: ground reaction forces
- Constraints: dynamics, swing leg forces = 0, friction cone

### 🦿 Controllers

- **Ground Controller**: maps MPC forces to joint torques using Jacobian transpose.
- **Swing Controller**: computes desired foot positions during swing phase.

---

## 📊 Results

Simulations are conducted in DartPy, evaluating:
- Trajectory tracking (COM position and orientation)
- Gait stability (trot, pace)
- Foot placement accuracy
- Control frequency and force constraints

---

## 🛠️ Known Issues

- Occasional ground penetration, problem arise in simulation side

---

## 📚 References

- [Di Carlo et al. (2018). Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control](https://ieeexplore.ieee.org/document/8594448)

---

## 👥 Authors

Vittorio Pisapia([VittorioPisapia](https://github.com/VittorioPisapia)), Jacopo Tedeschi([jacopotdsc](https://github.com/jacopotdsc)), Emiliano Paradiso([Emilianogith ](https://github.com/Emilianogith)), Brian Piccione([HatoKng](https://github.com/HatoKng))
