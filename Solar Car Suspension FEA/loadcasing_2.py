## Introduction ##

# Buckeye Solar Racing Suspension Load Casing Tool - By Mehmet Kara
# Solves suspension component loads.
# Read the README!

## CODE START ##

# Import libraries
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define vehicle parameters for 2G bump case (user adjustable)
m = 319 # vehicle mass [kg]
g = 9.81 # gravitational constant [m/s^2]
W = m * g # vehicle weight [N]
WB = 1.518 # vehicle wheelbase [m]
TW = (558.575 * 2) / 1000 # vehicle trackwidth [m]
hcg = 0.318 # CG height [m]
a = 0.6437 # distance from front axle to CG [m]
Wf = W * (1 - (a / WB)) # calculates weight on the front axle [N]
Wr = W - Wf; # calculates weight on the rear axle [N]

# Load CarSim data
data_file = './WC Forces & Moments Data/corneringandbrakingdata.csv' # data file directory (user defined)
T = pd.read_csv(data_file) # reads the data file
Ax = T['Ax'].to_numpy() # saves the long accel
Ay = T['Ay'].to_numpy() # saves the lateral accel
# Cornering load case snapshot ( Ay ~= 1G and Ax ~= 0G)
idxC = int(np.argmin(np.abs(np.aps(Ay) - 1.0) + np.abs(Ax))) # saves cornering index
print(f"Cornering snapshot @ t = {T['Time'].iloc[idxC]:.3f} s "
      f"(Ax={Ax[idxC]:.3f}G, Ay={Ay[idxC]:.3d}G)")
# Braking load case snapshot (Ay ~= 0G and Ax ~= 1G)
idxB = int(np.argmin(np.abs(np.abs(Ax) - 1.0) + np.abs(Ax)))
print(


