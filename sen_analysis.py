# %% Libraries
import numpy as np
from scipy.integrate import solve_ivp
import itertools
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mcolors

plt.rcParams["text.usetex"] = True
plt.rcParams["font.family"] = "serif"
plt.rcParams["font.serif"] = ["Computer Modern"]

import os
os.chdir("...")
print(os.getcwd())
print(os.listdir())
# Local import
from Libraries.DC_Motor import DC_motor
from Libraries.Regulator import Regulator

# %% Model Params
motor_params = {
    "R": (0.2, 0.04), 
    "L": (0.5, 0.1),
    "Kb": (1.0, 0.005),
    "Kt": (1.0, 0.005),
    "J": (2.0, 0.002),
    "B": (0.05, 0.0002)
}

v_max, v_min = 120, -120
duration = 220
dt = 0.01
steps = np.arange(0, duration + dt, dt)
ref = np.full_like(steps, 0, dtype=float)
ref[1000:4500] = 50 
ref[4500:8000] = 100 
ref[8000:11500] = -50
ref[11500:15000] = -100
ref[15000:] = 100

TL = np.zeros_like(steps, dtype=float)
TL[18500:] = 60

model = DC_motor(motor_params["R"][0], 
                 motor_params["L"][0], 
                 motor_params["Kb"][0], 
                 motor_params["Kt"][0], 
                 motor_params["J"][0],
                 motor_params["B"][0],
                 TL = TL)

Kp_tuned = 0.632
Ki_tuned = 0.316
Kd_tuned = 0.316

# RL Grid generation
R_gen = np.linspace(motor_params["R"][0] - 3*motor_params["R"][1], 
                    motor_params["R"][0] + 3*motor_params["R"][1], 
                    30)

L_gen = np.linspace(motor_params["L"][0] - 3*motor_params["L"][1], 
                    motor_params["L"][0] + 3*motor_params["L"][1], 
                    30)

R_vec = [round(x, 2) for x in R_gen]
L_vec = [round(x, 2) for x in L_gen]

grid = list(itertools.product(R_vec, L_vec))

# %% GMPID
alpha = np.arange(0,1,0.1)
os.makedirs("Sensitivity_RL", exist_ok = True)

for k, l in enumerate(alpha):
    C_GMPID = Regulator(Kp = Kp_tuned, 
                       Ki = Ki_tuned, 
                       Kd = Kd_tuned, 
                       alpha=l, 
                       beta_r = 1.0,
                       beta_f = 1.0,
                       Lim = (v_min, v_max))
    C_GMPID.reset()
    J_cost = np.zeros(len(grid), dtype=float)
    data = []

    for j, x in enumerate(grid):
        print(f"Iter: {j+1}, R: {x[0]}, L: {x[1]}")

        model = DC_motor(x[0], 
                    x[1], 
                    motor_params["Kb"][0], 
                    motor_params["Kt"][0], 
                    motor_params["J"][0],
                    motor_params["B"][0],
                    TL = TL)

        state = np.zeros((2, len(steps)), dtype = float)
        E = np.zeros_like(ref, dtype=float)
        for i in range(len(steps)-1):
            GMPID = C_GMPID.control(mea = state[1, i],
                                    ref = ref[i],
                                    dt = dt)
            sol = solve_ivp(model.ODE, (steps[i], steps[i]+dt), state[:,i], args = (GMPID, TL[i]))
            E[i] = abs(ref[i] - state[1, i])
            state[:, i+1] = sol.y[:, -1]
        C_GMPID.reset()
        J_cost[j] = (steps @ E) * dt
        save = {"R": x[0], 
                "L": x[1], 
                "cost": J_cost[j]}
        data.append(save)
        print(f"Iter: {j+1}, Done! \n")
    # out_csv = os.path.join("Sensitivity_RL", f"GMPID_sensitivity_{k}.csv")
    # pd.DataFrame(data).to_csv(out_csv, index = True)
    # print(f"Save csv file for alpha = {l} \n")

# %% PID
C_PID = Regulator(Kp = Kp_tuned, 
                   Ki = Ki_tuned, 
                   Kd = Kd_tuned, 
                   alpha=0.0, 
                   beta_r = 0.0,
                   beta_f = 0.0, 
                   Lim = (v_min, v_max))
C_PID.reset()

J_cost_pid = np.zeros(len(grid), dtype=float)
data_pid = []

for j, x in enumerate(grid):
    print(f"Iter: {j+1}, R: {x[0]}, L: {x[1]}")

    model = DC_motor(x[0], 
                 x[1], 
                 motor_params["Kb"][0], 
                 motor_params["Kt"][0], 
                 motor_params["J"][0],
                 motor_params["B"][0],
                 TL = TL)

    state_pid = np.zeros((2, len(steps)), dtype = float)
    E_pid= np.zeros_like(ref, dtype=float)
    for i in range(len(steps)-1):
        PID = C_PID.control(mea = state_pid[1, i],
                                 ref = ref[i],
                                 dt = dt)
        sol = solve_ivp(model.ODE, (steps[i], steps[i]+dt), state_pid[:,i], args = (PID, TL[i]))
        E_pid[i] = abs(ref[i] - state_pid[1, i])
        state_pid[:, i+1] = sol.y[:, -1]
    C_PID.reset()
    J_cost_pid[j] = (steps @ E_pid) * dt
    save = {"R": x[0], 
            "L": x[1], 
            "cost": J_cost_pid[j]}
    data_pid.append(save)
    print(f"Iter: {j+1}, Done! \n")

# os.makedirs("Sensitivity_RL", exist_ok = True)
# out_csv = os.path.join("Sensitivity_RL", "PID_sensitivity.csv")
# pd.DataFrame(data_pid).to_csv(out_csv, index = True)

# %% Manual importing data
pid_s = pd.read_csv("Sensitivity_RL/PID_sesitivity.csv").values
gmpid_s0 = pd.read_csv("Sensitivity_RL/GMPID_sensitivity_0.csv").values
gmpid_s1 = pd.read_csv("Sensitivity_RL/GMPID_sensitivity_1.csv").values
gmpid_s2 = pd.read_csv("Sensitivity_RL/GMPID_sensitivity_2.csv").values
gmpid_s3 = pd.read_csv("Sensitivity_RL/GMPID_sensitivity_3.csv").values
gmpid_s4 = pd.read_csv("Sensitivity_RL/GMPID_sensitivity_4.csv").values
gmpid_s5 = pd.read_csv("Sensitivity_RL/GMPID_sensitivity_5.csv").values
gmpid_s6 = pd.read_csv("Sensitivity_RL/GMPID_sensitivity_6.csv").values
gmpid_s7 = pd.read_csv("Sensitivity_RL/GMPID_sensitivity_7.csv").values
gmpid_s8 = pd.read_csv("Sensitivity_RL/GMPID_sensitivity_8.csv").values
gmpid_s9 = pd.read_csv("Sensitivity_RL/GMPID_sensitivity_9.csv").values

# %% Data handling
base = 180383.52129123762 # PID error from nominal tracking test

pid_s_nom = pid_s[:, -1]/base
S_max_pid = np.max(pid_s_nom)
S_mean_pid = np.mean(pid_s_nom)

gmpid_s = np.array((gmpid_s0[:, -1], gmpid_s1[:, -1], gmpid_s2[:, -1], 
                    gmpid_s3[:, -1], gmpid_s4[:, -1], gmpid_s5[:, -1], 
                    gmpid_s6[:, -1], gmpid_s7[:, -1], gmpid_s8[:, -1], 
                    gmpid_s9[:, -1])).T
gmpid_s_nom = gmpid_s/base

all_s = np.concatenate((np.hstack(gmpid_s_nom), pid_s_nom))

gmpid_min = np.zeros(gmpid_s_nom.shape[0])
for i in range(gmpid_s_nom.shape[0]):
        gmpid_min[i] = min(gmpid_s_nom[i,:])

gmpid_s_nom_stack = np.hstack(gmpid_min)
S_max_gmpid = np.max(gmpid_s_nom_stack)
S_mean_gmpid = np.mean(gmpid_s_nom_stack)

delta_s_max = (S_max_pid - S_max_gmpid)/S_max_pid
delta_s_mean = (S_mean_pid - S_mean_gmpid)/S_mean_pid

gmpid_min_grid = gmpid_min.reshape((30,30)).T

gmpid_04 = (gmpid_s4[:, -1]/base).reshape((30, 30)).T
gmpid_05 = (gmpid_s5[:, -1]/base).reshape((30, 30)).T
gmpid_06 = (gmpid_s6[:, -1]/base).reshape((30, 30)).T

# %% Visualize
pid_s_nom = (pid_s[:, -1]/base).reshape((30,30)).T # row is L, column is R
L_vec = pid_s[:30, 2]
R_vec = pid_s[0::30, 1]

R_g, L_g = np.meshgrid(R_vec, L_vec)

fig, ax = plt.subplots(1, 2, figsize = (8.4, 3.8), 
                       sharey = True, 
                       sharex = True,
                       constrained_layout = True)
cmap = plt.cm.Blues
norm = mcolors.Normalize(vmin =  min(all_s), 
                         vmax = max(all_s))

ax[0].pcolormesh(R_g, 
              L_g, 
              pid_s_nom, 
              shading="auto", 
              cmap = cmap, 
              norm = norm)
cs0 = ax[0].contour(R_g, L_g, pid_s_nom, 
              levels=6, 
              linewidths=0.6, 
              colors="k", alpha=0.5)
ax[0].set_xlabel(r"$R$", fontsize = 20)
ax[0].set_ylabel(r"$L$", fontsize = 20)
ax[0].tick_params(axis = "both", labelsize = 20)
ax[0].set_title(r"$\mathrm{PID}$", fontsize = 20)

ax[1].pcolormesh(R_g, 
              L_g, 
              gmpid_min_grid, 
              shading="auto", 
              cmap = cmap, 
              norm = norm)
ax[1].set_xlabel(r"$R$", fontsize = 20)
ax[1].tick_params(axis = "both", labelsize = 20)
cs1 = ax[1].contour(R_g, L_g, gmpid_min_grid, 
              levels=6, 
              linewidths=0.6, 
              colors="k", alpha=0.5)
ax[1].set_title(r"$\mathrm{GM}$-"r"$\mathrm{PID}$", fontsize = 20)

sm = cm.ScalarMappable(norm = norm, cmap = cmap)
sm.set_array([])

cbar = fig.colorbar(sm, ax = ax,
                    fraction = 0.04, 
                    pad = 0.02,
                     location = "right")
cbar.set_label(r"$S(R,L)$", fontsize = 16)
cbar.ax.tick_params(labelsize = 16)

ax[0].clabel(cs0, inline=True, fontsize=16)
ax[1].clabel(cs1, inline=True, fontsize=16)

# plt.savefig("/Users/nyinyia/Documents/09_LSU_GIT/CDC_iPID/plots/sen_PIDvGMPID_v2.pdf", 
#             bbox_inches = "tight", 
#             dpi = 300)

# %% Visualize 2
pid_s_nom = (pid_s[:, -1]/base).reshape((30,30)).T # row is L, column is R
L_vec = pid_s[:30, 2]
R_vec = pid_s[0::30, 1]

R_g, L_g = np.meshgrid(R_vec, L_vec)

fig, ax = plt.subplots(2, 2, figsize = (8.4, 7.2), 
                       sharey = True, 
                       sharex = True,
                       constrained_layout = True)
cmap = plt.cm.Blues
norm = mcolors.Normalize(vmin =  min(all_s), 
                         vmax = max(all_s))

# PID
ax[0,0].pcolormesh(R_g, 
              L_g, 
              pid_s_nom, 
              shading="auto", 
              cmap = cmap, 
              norm = norm)
cs0 = ax[0,0].contour(R_g, L_g, pid_s_nom, 
              levels=6, 
              linewidths=0.6, 
              colors="k", alpha=0.5)
# ax[0,0].set_xlabel(r"$R$", fontsize = 20)
ax[0,0].set_ylabel(r"$L$", fontsize = 20)
ax[0,0].tick_params(axis = "both", labelsize = 20)
ax[0,0].set_title(r"$\mathrm{PID}$", fontsize = 20)

# alpha = 0.4
ax[0,1].pcolormesh(R_g, 
              L_g, 
              gmpid_04, 
              shading="auto", 
              cmap = cmap, 
              norm = norm)
# ax[0,1].set_xlabel(r"$R$", fontsize = 20)
ax[0,1].tick_params(axis = "both", labelsize = 20)
cs1 = ax[0,1].contour(R_g, L_g, gmpid_04, 
              levels=6, 
              linewidths=0.6, 
              colors="k", alpha=0.5)
ax[0,1].set_title(r"$\alpha = 0.4$", fontsize = 20)

# alpha = 0.5
ax[1,0].pcolormesh(R_g, 
              L_g, 
              gmpid_05, 
              shading="auto", 
              cmap = cmap, 
              norm = norm)
ax[1,0].set_ylabel(r"$L$", fontsize = 20)
ax[1,0].set_xlabel(r"$R$", fontsize = 20)
ax[1,0].tick_params(axis = "both", labelsize = 20)
cs2 = ax[1,0].contour(R_g, L_g, gmpid_05, 
              levels=6, 
              linewidths=0.6, 
              colors="k", alpha=0.5)
ax[1,0].set_title(r"$\alpha = 0.5$", fontsize = 20)

# alpha = 0.6
ax[1,1].pcolormesh(R_g, 
              L_g, 
              gmpid_06, 
              shading="auto", 
              cmap = cmap, 
              norm = norm)
ax[1,1].set_xlabel(r"$R$", fontsize = 20)
ax[1,1].tick_params(axis = "both", labelsize = 20)
cs3 = ax[1,1].contour(R_g, L_g, gmpid_06, 
              levels=6, 
              linewidths=0.6, 
              colors="k", alpha=0.5)
ax[1,1].set_title(r"$\alpha = 0.6$", fontsize = 20)

sm = cm.ScalarMappable(norm = norm, cmap = cmap)
sm.set_array([])

cbar = fig.colorbar(sm, ax = ax,
                    fraction = 0.05, 
                    pad = 0.025,
                     location = "right")
cbar.set_label(r"$S(R,L)$", fontsize = 16)
cbar.ax.tick_params(labelsize = 16)

ax[0,0].clabel(cs0, inline=True, fontsize=16)
ax[0,1].clabel(cs1, inline=True, fontsize=16)
ax[1,0].clabel(cs2, inline=True, fontsize=16)
ax[1,1].clabel(cs3, inline=True, fontsize=16)

# plt.savefig("/Users/nyinyia/Documents/09_LSU_GIT/CDC_iPID/plots/sen_PIDvGMPID_v3.pdf", 
#             bbox_inches = "tight", 
#             dpi = 300)

# %%