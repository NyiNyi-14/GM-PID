# %%
import numpy as np

# %%
class Regulator:
    def __init__(self, 
                Kp: float,
                Ki: float, 
                Kd: float, 
                alpha = 0.0,
                beta_r = 1.0,
                beta_f = 1.0, 
                Lim = (None, None)):
                
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.alpha = alpha
        self.beta_r = beta_r
        self.beta_f = beta_f
        self.Lim = Lim
        self.integral = 0
        self.prev_error = 0
        self.prev_meas = 0
        self.prev_ref = 0
 
    def control(self, mea, ref, dt):
        error = ref - mea
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        d_mea = (mea - self.prev_meas) / dt
        self.prev_meas = mea
        d_ref = (ref - self.prev_ref) / dt
        self.prev_ref = ref

        PID = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        GMPID = (self.beta_r*d_ref - self.beta_f*d_mea + PID)/(1-self.alpha)
        action = np.clip(GMPID, self.Lim[0], self.Lim[1])
        
        return action

    def reset(self):
        self.integral = 0
        self.prev_error = 0
        self.prev_meas = 0
        self.prev_ref = 0

# %%
