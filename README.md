# Gain-Modulated PID Control for Robust Performance Under Parametric Uncertainty

<p align="center">
  <b> Block diagram of the proposed GM-PID controller</b><br>
  <img src="media/Model_Diagram.png" width="100%">
</p>

This work introduces a framework designed to shape closed-loop error dynamics without explicit disturance estmation by embeding unknown effects into control law through a gain modulated mechanism. This repostitory contains the code, data, and methods used to model a gain-modulated PID control method. The figure above extends the conventional PID structure (red) with derivative scaling terms and an α-dependent gain modulation factor. 

## Project Structure

  - The `Libraries` folder contains custom modules for system modeling, parameter estimation, and GMPID design used by the main scripts.

  - `nom_tracking.py` – Runs the DC Motor simulation shown in the figure above. Results are analyzed on the specified parameter $\alpha$, performance is measured by the tracking cost, $J$, normalized by a PID simulation of the same system. 

  - `sen_analysis.py` – Comparison of PID and GM-PID performance over (R,L) variation 

  - `sin_loadtorque.py` – Analysing sinusoidal disturbance rejection performance of PID and GM-PID.

---

## Requirements

- control==0.10.1
- matplotlib==3.10.8
- numpy==2.4.2
- pandas==3.0.1
- scipy==1.17.1

Install dependencies using:

```bash
pip install -r requirements.txt
```
---

## How to Run

### Setup

Download or clone the repository:

```bash
git clone https://github.com/NyiNyi-14/GM-PID.git
```

Make sure all scripts are in the same directory. 

---

## Results

<p align="center">
  <b>Response of classical PID controller:</b><br>
  <img src="media/PID_Input_Response.png" width="80%"><br>

  Speed tracking and control input response of the classical PID controller under the step reference profile. The actuator saturation limits are indicated for reference. <br>

</p>
<p align="center">
  <b>Response of Gain-Modulated PID controller:</b><br>
  <img src="media/GMPID_Input_Response.png" width="80%"><br>
  
  Parametric response of the GM-PID controller under different modulation factors α. The color bar represents the value of α, illustrating its influence on transient and steady-state behavior.<br>
</p>

<p align="center">
  <b>Normalized performance index and speed responses:</b><br>
  <img src="media/Normalized_performance.png" width="80%"><br>

  (Top) Normalized performance index J_rel for PID and GM-PID configurations. <br>
  (Bottom) Representative GM-PID speed responses for α = 0.3 and α = 0.5 are shown below to illustrate the performance variation induced by the modulation parameter α.<br>
</p>

<p align="center">
  <b>Sensitivity of PID and GM-PID over (R,L) uncertainty domains</b><br>

  <img src="media/Sensitivity_Score_Plots.png" width="80%"><br>
</p>

<p align="center">
  <b>Dynamic load torque profile and Speed Responses </b><br>
  <img src="media/Dynamic_Load_Torque.png" width="80%"><br>
  
  Dynamic load torque profile and speed responses of PID and GM-PID under a sinusoidal load disturbance applied at t = 40 s. <br>
</p>

<p align="center">
  <b>Response of classical PID controller:</b><br>
  <img src="media/performance.png" width="80%"><br>

  (Top) Performance index Jrel for PID and GM-PID across tested α values. <br>
  (Bottom) Representative GM-PID responses for α = 0.3 and α = 0.9.<br>

</p>
---

## Related Work

This project focus on:

- Robust controller
- Sensitivity analysis

---

## Citation

To acknowledge the use of this work, please cite the following publication:

```bibtex
To be updated upon publication
```
---
