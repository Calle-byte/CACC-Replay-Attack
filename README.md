# Replay Attack Simulation in a CACC Vehicle Platoon

This repository contains the MATLAB implementation for simulating a 6-vehicle platoon equipped with Cooperative Adaptive Cruise Control (CACC), evaluating its **nominal dynamics**, its response to a **Replay Attack** on Vehicle 2 and the implementation of a **Detector**.

> **Reference Paper:**  
> R. Merco, Z. A. Biron, and P. Pisu, *"Replay Attack Detection in a Platoon of Connected Vehicles with Cooperative Adaptive Cruise Control"*, 2018 Annual American Control Conference (ACC), Milwaukee, WI, USA, 2018, pp. 5582-5587.


## Repository Structure

- `code/main_Platoon_Replay_Attack.m`: Main script to execute the nominal CACC simulation.
- `code/Inizialization_Platoon_var.m`: Initializes simulation parameters, vehicle states, and initial conditions.
- `code/Dinamic_NoAttack.m`: Simulates the nominal discrete-time vehicle dynamics and explicit CACC control law.
- `code/Dinamic_ReplayAttack.m`: Simulates the platoon dynamics under a Replay Attack on Vehicle 2 starting at t = 30sec.
- `code/Plot_Platoon.m`: Generates plots for velocities, inter-vehicle distances, spacing errors, relative distances, and accelerations.
- `code/Detector.m`: ***.

---

## How to Run

Clone or download this repository:
   ```bash
   git clone [(https://github.com/Calle-byte/CACC-Replay-Attack)](https://github.com/Calle-byte/CACC-Replay-Attack)
