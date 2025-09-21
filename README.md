# Inverted Pendulum on a Cart – MPC + EKF

This project implements a **real hardware inverted pendulum on a cart**, controlled by a Raspberry Pi 5.  
All mechanical parts were **3D-printed and designed from scratch**, and the control software is written in **Python** with multiprocessing for parallel control loops.



## Features
- Extended Kalman Filter (EKF) with outlier rejection (based on Mahalanobis distance) for state estimation and sensor filtering
- Real-time linear Model Predictive Control (MPC) for pendulum stabilization  
- Multiprocessing Python architecture for simultaneous control & sensor loops  
- I²C sensors and DC motor control via H-bridge  
- MATLAB-based system identification to derive dynamic model parameters 

## Hardware
- Raspberry Pi 5  
- DC motor + DRV8871 H-bridge
- 14.8V LiPo battery  
- AS5600 12-bit magnetic encoders (pendulum + cart)  
- Limit switches for calibration & safety  
- Custom 3D-printed cart & pendulum structure
- Aluminium 20x20 profile + rollers  

## Software
- Python (NumPy, CasADi, ACADOS, multiprocessing)  
- MATLAB (system identification)

## Development Setup
- Runs on **Raspberry Pi 5** under **Linux**  
- All interaction through **SSH and terminal** (no desktop environment)  
- Real-time multiprocessing processes pinned to CPU cores using `os.sched_setaffinity`  
- Uses shared memory for fast inter-process communication


[▶ Watch the demo video](demo/inverted_pendulum_demo.mp4)  
