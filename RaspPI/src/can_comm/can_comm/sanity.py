#                dy : list = self.curr_currents[0:] - self.prev_currents[0:]
import numpy as np
curr = np.array([1, 4, 6, 8])
prev = np.array([1, 2, 3, 4])
out = curr[0:] / prev[0:]
flag = out[0:] == 1