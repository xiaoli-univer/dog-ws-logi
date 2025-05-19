from math import cos, sin

import matplotlib.pyplot as plt
import numpy as np

# Set simulation time
T = 1000
dt = 0.001

# Set desired trajectory
xd = np.sin(np.linspace(0, 10, T))
dxd = np.cos(np.linspace(0, 10, T))
ddxd = -np.sin(np.linspace(0, 10, T))

# Set impedance parameters
Md = 1
Bd = 10
Kd = 400

# Set initial conditions
x0 = 0
dx0 = 0

# Initialize variables
x = np.zeros(T)
dx = np.zeros(T)
f = np.zeros(T)
x[0] = x0
dx[0] = dx0

# Simulate impedance control
for i in range(T - 1):
    # Compute desired end-effector acceleration
    ddx_d = np.linalg.solve(Md, (f[i] - Bd * (dx[i] - dxd[i]) - Kd * (x[i] - xd[i])))
    
    # Update end-effector velocity and position
    dx[i+1] = dx[i] + ddx_d * dt
    x[i+1] = x[i] + dx[i+1] * dt

# Plot results
plt.figure(figsize=(8, 6))
plt.subplot(2, 1, 1)
plt.plot(x)
plt.plot(xd)
plt.title('Position')
plt.subplot(2, 1, 2)
plt.plot(dx)
plt.plot(dxd)
plt.title('Velocity')
plt.tight_layout()
plt.show()
 