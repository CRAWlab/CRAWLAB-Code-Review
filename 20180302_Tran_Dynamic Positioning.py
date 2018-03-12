#! /usr/bin/env python 

###############################################################################
# dp_control.py
#
# A dynamic positioning controller to take a reference
# trajectory and follow it with minimal error 
#
# Created: 01/24/2018
#   - Diana Tran
#   - dtt8105@louisiana.edu
#
###############################################################################
import numpy as np # General numerics
from scipy.integrate import odeint # Integration
from scipy.optimize import minimize # Optimization
import matplotlib.pyplot as plt # Plotting

# Define System Parameters
m = 23                  # mass (kg)
Xu_dot = -2.0           # kg
Yv_dot = -10.0          # kg
xg = 0.046              # x distance to COG (m)
Yr_dot = -0.0           # kgm
Iz = 1.76               # moment of inertia about z-axis (kgm^2)
Nr_dot = -1.0           # kgm^2/s
Xu = - 0.72253          # kg/s
Yv = - 0.88965          # kg/s
Yr = - 7.250            # kgm/s
Nv = 0.0313             # kgm/s
Nr = -1.900             # kgm^2/s
ly1 = 3.0
lx1 = 3.0
ly2 = 3.0
lx2 = 3.0

# Initial Position
x_init = 0.0                      # Initial surge position                 
x_dot_init = 0.0                  # Initial surge velocity
y_init = 0.0                      # Initial sway position  
y_dot_init = 0.0                  # Initial sway velocity
psi_init = 0.0                    # Initial yaw/heading angle position   
psi_dot_init = 0.0                # Initial yaw rate

# Initial Guess
alpha1_init = 30.0
alpha2_init = 90.0

# ODE solver parameters
abserr = 1.0e-9
relerr = 1.0e-9
max_step = 0.01
stoptime = 25
numpoints = 5001

# Set up time array
t = np.linspace(0.0, stoptime, numpoints)

# Set up the parameters for the input function
StartTime = 0.5            # Time the f(t) input will begin
T_amp = 2.0                # Amplitude of Disturbance force (N)

# Target trajectory
y_d = (t/5)**2
x_d = t/3**1.5

# This function defines the dynamics of our vessel, the equations of motion
# and the rate at which it changes position.
def dp_model(w, t, p):
   
    x, x_dot, y, y_dot, psi, psi_dot = w
    m, Xu_dot, Yv_dot, xg, Yr_dot, Iz, Nr_dot, Xu, Yv, Yr, Nv, Nr, StartTime, T_amp, alpha1, alpha2 = p

    # Create sysODE = (x, x_dot, y, y_dot, psi, psi_dot)
    sysODE = [x_dot,
              ((np.cos(psi) - np.sin(psi)) + tau_x(alpha1, alpha2, t, p) - (Xu * x_dot)) / (m-Xu_dot),
              y_dot,
              ((np.sin(psi) + np.cos(psi)) + tau_y(alpha1, alpha2, t, p) - ((-Yv - Yr) * y_dot)) / ((m - Yv_dot) + ((m * xg) - Yr_dot)),
              psi_dot,
              (tau_n(alpha1, alpha2, t, p) - ((-Nv - Nr)* psi_dot)) / (((m * xg) - Yr_dot) + (Iz - Nr_dot))]
              
    return sysODE

def tau_x(alpha1, alpha2, t, p):
    
    thrust_x=np.cos(alpha1)*F1(t,p) + np.cos(alpha2)*F2(t,p)
    
    return thrust_x
     
def tau_y(alpha1, alpha2, t, p):
    
    thrust_y=np.sin(alpha1)*F1(t,p) + np.sin(alpha2)*F2(t,p)
    
    return thrust_y
    
def tau_n(alpha1, alpha2, t, p):
    
    thrust_n=(-ly1*np.cos(alpha1) + lx1*np.sin(alpha1))*F1(t,p) + (-ly1*np.cos(alpha2) + lx1*np.sin(alpha2))*F2(t,p)
    
    return thrust_n

def F1(t,p):
    m, Xu_dot, Yv_dot, xg, Yr_dot, Iz, Nr_dot, Xu, Yv, Yr, Nv, Nr, StartTime, T_amp, alpha1, alpha2 = p
    f = T_amp * (t >= StartTime) * (t <= StartTime + 0.5)
    
    return f

def F2(t,p):
    m, Xu_dot, Yv_dot, xg, Yr_dot, Iz, Nr_dot, Xu, Yv, Yr, Nv, Nr, StartTime, T_amp, alpha1, alpha2 = p
    g = T_amp * (t >= StartTime) * (t <= StartTime + 0.5)
    
    return g

w0 = [x_init, x_dot_init, y_init, y_dot_init, psi_init, psi_dot_init]
    
alpha0 = [alpha1_init,alpha2_init]

def objective(X):
    
    alpha1,alpha2 = X
    p = [m, Xu_dot, Yv_dot, xg, Yr_dot, Iz, Nr_dot, Xu, Yv, Yr, Nv, Nr, StartTime, T_amp, alpha1, alpha2]    
    resp = odeint(dp_model,w0,t,args=(p,))
    error = np.sum((resp[:,0]-x_d[:])**2 + (resp[:,2]-y_d[:])**2)
    
    return error

res = minimize(objective,alpha0,options={'disp':True})
print(res)
alpha1_opt=res.x[0]
alpha2_opt=res.x[1]

print(alpha1_opt)
print(alpha2_opt)

w0 = [x_init, x_dot_init, y_init, y_dot_init, psi_init, psi_dot_init]

p = [m, Xu_dot, Yv_dot, xg, Yr_dot, Iz, Nr_dot, Xu, Yv, Yr, Nv, Nr, StartTime, T_amp, alpha1_opt, alpha2_opt] 

resp = odeint(dp_model,w0,t,args=(p,))

# Plot the surge and sway response
# Set the plot size - 3x2 aspect ratio is best
fig = plt.figure(figsize=(6, 4))
ax = plt.gca()
plt.subplots_adjust(bottom=0.17, left=0.17, top=0.96, right=0.96)

# Change the axis units to serif
plt.setp(ax.get_ymajorticklabels(),family='serif',fontsize=18)
plt.setp(ax.get_xmajorticklabels(),family='serif',fontsize=18)

ax.spines['right'].set_color('none')
ax.spines['top'].set_color('none')

ax.xaxis.set_ticks_position('bottom')
ax.yaxis.set_ticks_position('left')

# Turn on the plot grid and set appropriate linestyle and color
ax.grid(True,linestyle=':',color='0.75')
ax.set_axisbelow(True)

# Define the X and Y axis labels
plt.xlabel('Time (s)', family='serif', fontsize=22, weight='bold', labelpad=5)
plt.ylabel('Position (m)', family='serif', fontsize=22, weight='bold', labelpad=10)

# Plot the first element of resp for all time. It corresponds to the position.
plt.plot(t, resp[:,0], linewidth=2, linestyle = '-', label=r'Surge')
plt.plot(t, resp[:,2], linewidth=2, linestyle = '-', label=r'Sway')
plt.plot(t, x_d[:], linewidth=2, linestyle = '-', label=r'Surge Desired')
plt.plot(t, y_d[:], linewidth=2, linestyle = '-', label=r'Sway Desired')

# plt.xlim(0,25)
# plt.ylim(0,10)

# # Create the legend, then fix the fontsize
leg = plt.legend(loc='upper right', ncol = 2, fancybox=True)
ltext  = leg.get_texts()
plt.setp(ltext,family='serif',fontsize=18)

# Adjust the page layout filling the page using the new tight_layout command
plt.tight_layout(pad = 0.5)

plt.show() 
