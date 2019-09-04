import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['CONSTANT_SPEED'] = False

# weight used in measure_and_update
# should be >0 (if ==0 always take v =v_0 = 0)
# if == 1 self.v is always just set to the measurement (and so responds to changes in v quickly)
# if small, it responds slowly to changes in speed
# the effects on response time trade off against those on the errors - if you put 100% weight in new measurements
# then you cant reduce the error by encorporating old measurements.

weight = 0.5

class KalmanFilterToy:
    def __init__(self):
        self.v = 0
        self.prev_x = 0
        self.prev_t = 0
    def predict(self,t):
        # return "prediction" which is the current x state
        dt = t - self.prev_t 
        prediction = self.prev_x + self.v * dt
        return prediction
    def measure_and_update(self,x,t):
        dt = t - self.prev_t
        dx = x - self.prev_x
        measured_v = dx / dt
        self.v += weight * (measured_v - self.v)
        self.prev_x = x
        self.prev_t = t
        return


sim_run(options,KalmanFilterToy)
