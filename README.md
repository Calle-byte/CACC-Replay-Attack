# Replay Attack Simulation in a CACC Vehicle Platoon

This repository contains the MATLAB implementation for simulating a 6-vehicle platoon equipped with Cooperative Adaptive Cruise Control (CACC), evaluating both its **nominal dynamics** and its response to a **Replay Attack** on Vehicle 2[cite: 1, 5].

> **Reference Paper:**  
> R. Merco, Z. A. Biron, and P. Pisu, *"Replay Attack Detection in a Platoon of Connected Vehicles with Cooperative Adaptive Cruise Control"*, 2018 Annual American Control Conference (ACC), Milwaukee, WI, USA, 2018, pp. 5582-5587[cite: 1].


## Repository Structure

- `code/Inizializzazione_Platoon.m`: Initializes simulation parameters ($T_s, h, k_p, k_d$), vehicle states, and initial conditions.
- `code/Dinamica_Veicoli.m`: Simulates the nominal discrete-time vehicle dynamics and explicit CACC control law.
- `code/Dinamica_Veicoli_ReplayAttack.m`: Simulates the platoon dynamics under a Replay Attack on Vehicle 2 starting at $t = 30\text{s}$.
- `code/Plot_Risultati.m`: Generates plots for velocities, inter-vehicle distances, spacing errors, relative distances, and accelerations.
- `code/main.m`: Main script to execute the nominal CACC simulation.
- `code/main_replay.m`: Main script to execute the Replay Attack scenario.

---

## How to Run

1. Clone or download this repository:
   ```bash
   git clone [https://github.com/your-username/CACC-Replay-Attack-Simulation.git](https://github.com/your-username/CACC-Replay-Attack-Simulation.git)
