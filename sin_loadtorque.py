# %% Libraries
import numpy as np
from scipy.integrate import solve_ivp
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

# %% Model Parameters
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
ref = np.full_like(steps, 100, dtype=float)

T_off = np.full_like(steps, 20, dtype = float)
ome = 0.5
TL = T_off + 5 * np.sin(ome * steps)
TL[:4000] = 0
# plt.plot(steps, TL)

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

# %% PID
state_PID = np.zeros((2, len(steps)))
action_PID = np.zeros_like(ref, dtype=float)
error = np.zeros_like(ref, dtype=float)
C_PID = Regulator(Kp = Kp_tuned, 
                  Ki = Ki_tuned, 
                  Kd = Kd_tuned, 
                  alpha = 0, 
                  beta_r = 0,
                  beta_f = 0, 
                  Lim = (v_min, v_max))
C_PID.reset()
for i in range(len(steps)-1):
    PID = C_PID.control(mea = state_PID[1, i], ref = ref[i], dt = dt)
    sol = solve_ivp(model.ODE, (steps[i], steps[i] + dt), state_PID[:,i], args = (PID, TL[i]))
    state_PID[:, i+1] = sol.y[:, -1] 
    action_PID[i] = PID
    error[i] = abs(ref[i] - state_PID[1, i])
C_PID.reset()
cost = (steps @ error) * dt

# %% GMPID with alpha sweep
alpha = np.arange(0,1,0.1)

state_GMPID = np.zeros((alpha.shape[0], 2, len(steps)))
action_GMPID = np.zeros((alpha.shape[0], ref.shape[0]))
error_gm = np.zeros((alpha.shape[0], ref.shape[0]))
cost_gm = []

for j, k in enumerate(alpha):
    C_GMPID = Regulator(Kp = Kp_tuned, 
                        Ki = Ki_tuned, 
                        Kd = Kd_tuned, 
                       alpha = round(k, 2), 
                       beta_r = 1, 
                       beta_f = 1, 
                       Lim = (v_min, v_max))
    C_GMPID.reset()
    for i in range(len(steps)-1):
        GMPID = C_GMPID.control(mea = state_GMPID[j, 1, i], ref = ref[i], dt = dt)
        sol = solve_ivp(model.ODE, (steps[i], steps[i] + dt), state_GMPID[j, :, i], args = (GMPID, TL[i]))
        state_GMPID[j, :, i+1] = sol.y[:, -1]
        action_GMPID[j, i] = GMPID
        error_gm[j, i] = abs(ref[i] - state_GMPID[j, 1, i])
    C_GMPID.reset()
    J_c = (steps @ error_gm[j, :]) * dt
    cost_gm.append(J_c)

# %% Visualize 1
fig, ax = plt.subplots(3,1, figsize = (5,3), sharex = True, constrained_layout = True)

ax[0].plot(steps[:-1], TL[:-1], 
           color = "red", linewidth = 1.5,)
ax[0].grid(True, linestyle = "--", 
           linewidth = 0.6, alpha = 0.4)
ax[0].set_ylabel(r"$T^{*} \, \mathrm{[Nm]}$", fontsize = 14)
ax[0].tick_params(axis = "y", labelsize = 14, which = "major")

ax[1].plot(steps[:-1], state_PID[1,:-1], 
           color = "blue", linewidth = 1.5,)
ax[1].grid(True, linestyle = "--", 
           linewidth = 0.6, alpha = 0.4)
ax[1].set_ylabel(r"$\omega \, \mathrm{[rad/s]}$", fontsize = 14)
ax[1].tick_params(axis = "y", labelsize = 14, which = "major")
ax[1].text(
    145, 10,
    r"PID response",
    fontsize=14
)

cmap = plt.cm.Blues
norm = mcolors.Normalize(vmin = min(alpha), vmax = max(alpha))
for i in range(len(alpha)):
    colors = cmap(norm(alpha[i]))
    if i < (len(alpha)-1):
        ax[2].plot(steps[:-1], state_GMPID[i, 1, :-1], 
                color = colors, linewidth = 1.0, )
    if i == (len(alpha) - 1):
        ax[2].plot(steps[:-1], state_GMPID[i, 1, :-1], 
                color = colors, linewidth = 1.0,)
ax[2].grid(True, linewidth = 0.6, alpha = 0.4)
ax[2].set_ylabel(r"$\omega\, \mathrm{[rad/s]}$", fontsize = 14)
ax[2].set_xlabel(r"$t \, \mathrm{[s]}$", fontsize = 14)
ax[2].tick_params(axis = "both", 
                  which = "major", 
                  labelsize = 14)

ax[2].set_xlim(0, steps[-1])
ax[2].text(
    123, 10,
    r"GM-PID response",
    fontsize=14
)

sm = cm.ScalarMappable(norm = norm, cmap = cmap)
sm.set_array([])

cbar = fig.colorbar(sm, ax = ax,
                    fraction = 0.04, 
                    pad = 0.02,
                     location = "right")
cbar.set_label(r"$\alpha$", fontsize = 14)
cbar.ax.tick_params(labelsize = 14)

# plt.savefig("/Users/nyinyia/Documents/09_LSU_GIT/CDC_iPID/plots/Sine_T_V1.pdf", 
#             bbox_inches = "tight", 
#             dpi = 300)


# %% J vs alpha
J_base = np.array([cost/cost])
J_gmpid = np.array(cost_gm/cost)
J_tot = np.concatenate((J_base, J_gmpid))

xaxis = ["PID"]
for i in range(len(alpha)):
    xaxis.append(np.round(alpha[i], 2))

# Visualize 2
fig, ax = plt.subplots(3,1, figsize = (5, 4), constrained_layout = True)

ax[0].plot(xaxis, J_tot, 
           marker = "o", 
           markersize = 6, color = "blue", linewidth = 1.5)
ax[0].set_ylabel(r"$J_{\mathrm{rel}}$", fontsize = 14)
ax[0].set_xlabel(r"$\mathrm{PID}, \, \mathrm{GM}$-"r"$\mathrm{PID(\alpha)}$", fontsize =14)

ax[0].tick_params(axis = "both",
                  which = "major", 
                  labelsize = 14)
ax[0].grid(True, linestyle = "--", linewidth = 0.7, alpha = 0.4)
ax[0].set_xlim(-0.3, 10.3)

ax[1].plot(steps[:-1], state_GMPID[4, 1, :-1], 
           linewidth = 1.5, color = "blue",)
ax[1].set_ylabel(r"$\omega \, \mathrm{[rad/s]}$", fontsize = 14)
ax[1].grid(True, linestyle = "--", linewidth = 0.7, alpha = 0.4)
ax[1].tick_params(axis = "y", labelsize = 14)
ax[1].tick_params(labelbottom = False)
ax[1].text(
    185, 70,
    r"$\alpha = 0.3$",
    fontsize=14
)

ax[2].plot(steps[:-1], state_GMPID[-1, 1, :-1], 
           linewidth = 1.5, color = "blue", )
ax[2].sharex(ax[1])
ax[2].set_xlabel(r"$t \, \mathrm{[s]}$", fontsize = 14)
ax[2].set_ylabel(r"$\omega \, \mathrm{[rad/s]}$", fontsize = 14)
ax[2].grid(True, linestyle = "--", linewidth = 0.7, alpha = 0.4)
ax[2].set_xlim(0, steps[-1])
ax[2].tick_params(axis = "both", labelsize = 14)
ax[2].text(
    185, 65,
    r"$\alpha = 0.9$",
    fontsize=14
)

# plt.savefig("/Users/nyinyia/Documents/09_LSU_GIT/CDC_iPID/plots/J_T.pdf", 
#             bbox_inches = "tight", 
#             dpi = 300)

# %%