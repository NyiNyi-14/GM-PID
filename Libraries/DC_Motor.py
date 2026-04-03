# %% DC Motor Modelling 
class DC_motor:
    def __init__(self, R, L, Kb, Kt, J, B, TL = None):
        self.R = R
        self.L = L
        self.Kb = Kb
        self.Kt = Kt
        self.J = J
        self.B = B
        self.TL = TL

    def ODE(self, t, state, Va, TL):
        ia, omega = state
        ia_dt = (Va - self.R * ia - self.Kb * omega) / self.L
        omega_dt = (self.Kt * ia - self.B * omega - TL) / self.J
        return [ia_dt, omega_dt]
    
    def discrete(self, t, state, Va, TL, dt):
        ia, omega = state
        ia_dt = (Va - self.R * ia - self.Kb * omega) / self.L
        omega_dt = (self.Kt * ia - self.B * omega - TL) / self.J
        out_ia = ia + ia_dt * dt
        out_omega = omega + omega_dt * dt
        return [out_ia, out_omega]

# %%
