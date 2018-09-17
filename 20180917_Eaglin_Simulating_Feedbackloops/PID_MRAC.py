#! /usr/bin/env python

##########################################################################################
# PID_MRAC.py
#
# Uses MIT rule to linearize a simple nonlinear plant, then control using PID
#
#
#
# Created: 9/14/18
# Dynamics of the simple nonlinear plant are based on micro-electromagnetic actuator
# 		designed by Professor Hiroyuki Nabae
#
##########################################################################################




# Import modules and packages
import numpy as np
from RobotX_PID import PID
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import resp_characteristics as RC




# Define equations of motion of reference model
def eq_of_motion_ref(t, w):
    
    """
    Defines the differential equations for the reference model

    Arguments:
        t :  time
        w :  vector of the state variables:
    
    Returns:
        sysODE : A list representing the system of equations of motion as 1st order ODEs
    """

    # The state space equations
    x, x_dot = w
    M0, K0, C0, A, PID_current, alpha, x_bar, xg0 = p_ref

    sysODE_ref = [x_dot,
              # -k/m * x - c/m * x_dot + PID_current/m
              -K0/M0 * x - C0/M0 * x_dot + (A*alpha*(PID_current))/(M0*(x_bar+xg0)**2)
              ]

    return sysODE_ref



# Equation of motion of plant
def eq_of_motion_plant(t, w):
    
    """
    Defines the differential equations for the plant
    The system gain is set to 0.8 for now

    Arguments:
        t :  time
        w :  vector of the state variables:
    
    Returns:
        sysODE : A list representing the system of equations of motion as 1st order ODEs
    """

    # The state space equations
    x, x_dot = w
    # m, k, c, delta_k, con_input = p
    M0, K0, C0, A, PID_current, alpha, x_bar, xg0, MRAC.theta, delta_k = p
    xg=xg0-x/A

    sysODE_plant = [x_dot,
              # -k/m * x - c/m * x_dot - delta_k/m * x**2 + con_input/m
              # -K0/M0 * x - C0/M0 * x_dot - (delta_k*x**2)/M0 + (A*alpha*(PID_current))/(M0*(x_bar+xg0)**2) + (MRAC.theta*x**2)/M0
              (-K0 * x - C0 * x_dot - (delta_k*x**2) + (A*alpha*(PID_current))/((x_bar+xg0)**2) + (MRAC.theta*x**2))/M0
              
              # The unsimplified nonlinear system
              # -K0/M0 * x - C0/M0 * x_dot + (A*alpha*(PID_current))/(M0*(x_bar+xg)**2) + (MRAC.theta*x**2)/M0

              ]

    return sysODE_plant


class adjmech:
    # TODO: have it index t to figure out which element of ym to use
    
    def __init__(self,theta,ym,dt,gamma):
        self.theta=theta
        self.ym=ym
        self.dt=dt
        self.gamma=gamma
    
    def update(self,y,i):
        e=self.error(y,self.ym[i])
        
        # Integrate the new dtheta_dt and add to theta
        self.theta += -self.gamma*e*self.ym[i]*self.dt
    
    def error(self,y,ym):
        return y-ym


# PID gains without shaping
# kp=0    # proportional gain
# ki=10 # integral gain
# kd=0.1      # derivative gain
kp=1.8345    # proportional gain
ki=3334.0136 # integral gain
kd=6e-4      # derivative gain


delta_t = 1.0e-6      # sampling time
u_max=2.4             # maximum actuator effort
pid = PID(kp, ki, kd, delta_t, u_max, 0, 0.0)
pid_MRAC = PID(kp, ki, kd, delta_t, u_max, 0, 0.0)



# Define parameters for simulation
# Mechanical parameters

A=4 # amplification mechanism
M0= 0.1804666666666667
K0= 2089704.4266858997
C0= 0.0

xg0=50e-6
x_bar= 6.883065726984473e-05
alpha= 5.02654824574367e-06


# convert to match experiment (error measured in mm for experiments)
xg0=xg0*(10**3) # meters to mm
x_bar=x_bar*(10**3) # meters to mm
alpha=alpha*(10**3)**2 # meters^2 to mm^2
M0=M0*(10**-3) # kg to Mg
K0=K0*(10**-3) # N/m to M/mm
C0=C0*(10**-3) # meters to mm




# ODE solver parameters
abserr = 1.0e-9
relerr = 1.0e-9
max_step = 0.01
starttime = 0.0
# stoptime = 15
stoptime = 0.03
# stoptime = 50
# numpoints = 1001
# delta_t = 0.01

t = np.arange(starttime,stoptime,delta_t)
command_start=0.001   # The time at which the step input occurs
element_start=int(command_start/delta_t)  # element number at which command starts


# Define and pack up the parameters and initial conditions:
init=00.0 # initial actuator displacement
setpoint=100e-3 # desired final actuator displacement (mm)
desired_output = np.ones_like(t)*init
desired_output[element_start:6000]=25e-3
desired_output[6000:12000]=50e-3
desired_output[12000:18000]=75e-3
desired_output[18000:]=setpoint


command=desired_output

# initial conditions
x_init = 0.0
x_dot_init = 0.0
x0=[x_init, x_dot_init]
# Define initial condition to update at each iteration
x0_i=x0

# define array to store the response
ref_resp=np.zeros((2,len(t)))
ref_resp[:,0]=x0
pid_output_ref = np.zeros_like(t)


# simulate response of the reference model with PID control
for ii in range(len(t)-1):

	PID_current = pid.compute_output(command[ii], x0_i[0], t[ii+1], t[ii])
	pid_output_ref[ii] = PID_current # save current value for later plotting

	# pack the parameters and call the ode solver
	p_ref = [M0, K0, C0, A, PID_current, alpha, x_bar, xg0]
	solution = solve_ivp(eq_of_motion_ref, [t[ii], t[ii+1]], x0_i, 
	                 #dense_output=True,
	                t_eval=[t[ii],t[ii+1]], max_step=max_step, atol=abserr, rtol=relerr)

    # time array for this iteration
	sim_time_i = solution.t
    # sampled response
	resp_i = solution.y

    # add sampled response to array
	ref_resp[:,ii+1]=resp_i[:,-1]
    # redefine initial condition for next iteration
	x0_i=[solution.y[0,-1],solution.y[1,-1]]

# position of reference response to be stored as attribute in MRAC
x_ref_resp = ref_resp[0,:]




# Define parameters for MRAC
theta_init = 5000 # initial theta
# theta_init = 0 # initial theta
# gamma= 1e10 # adjustment gain
gamma= 3.0e10 # Illustrate possible instability
# gamma= 0 # adjustment gain


delta_k = 10000

# Define an instance of the adjmech class
MRAC=adjmech(theta_init,x_ref_resp,delta_t,gamma)

# Reset initial condition
x0_i=x0

plant_resp=np.zeros((2,len(t)))
plant_resp[:,0]=x0
pid_output_plant = np.zeros_like(t)

# simulate response of the nonlinear model with PID and MRAC
for ii in range(len(t)-1):

	PID_current = pid_MRAC.compute_output(command[ii], x0_i[0], t[ii+1], t[ii])
	pid_output_plant[ii] = PID_current # save current value for later plotting

	# pack the parameters and call the ode solver
	p = [M0, K0, C0, A, PID_current, alpha, x_bar, xg0, MRAC.theta, delta_k]
	solution = solve_ivp(eq_of_motion_plant, [t[ii], t[ii+1]], x0_i, 
	                 #dense_output=True,
	                t_eval=[t[ii],t[ii+1]], max_step=max_step, atol=abserr, rtol=relerr)
	sim_time_i = solution.t
	resp_i = solution.y

	plant_resp[:,ii+1]=resp_i[:,-1]
	x0_i=[solution.y[0,-1],solution.y[1,-1]]
	MRAC.update(solution.y[0,-1],ii+1)





# Plot the results
# Set the plot size - 3x2 aspect ratio is best
fig = plt.figure(figsize=(6,4))
ax = plt.gca()
plt.subplots_adjust(bottom=0.17,left=0.17,top=0.96,right=0.96)

# Change the axis units to CMUSerif-Roman
plt.setp(ax.get_ymajorticklabels(),family='CMUSerif-Roman')
plt.setp(ax.get_xmajorticklabels(),family='CMUSerif-Roman')


# Define the X and Y axis labels
plt.xlabel('Time (s)',family='CMUSerif-Roman',fontsize=22,weight='bold',labelpad=5)
plt.ylabel(r'Position (mm)',family='CMUSerif-Roman',fontsize=22,weight='bold',labelpad=10)

plt.plot(t,command, linestyle = '--', label=r'Command')
plt.plot(t, x_ref_resp, linestyle='-.', label=r'Reference')
plt.plot(t, plant_resp[0,:], linestyle='-', label=r'Response')
# plt.plot(shaped_time[0:(stoptime/deltaT)-1], shaped_response[0:len(shaped_time[0:(stoptime/deltaT)-1]),0], linewidth=2, linestyle='-', label=r'Input Shaping')
# uncomment below and set limits if needed
plt.xlim(0,0.030)
# plt.xlim(0,0.03)
plt.ylim(-0.001,0.11)

# Create the legend, then fix the fontsize
leg = plt.legend(loc='upper left', ncol=1,handlelength=1.5,handletextpad=1.1, labelspacing=0.25)

ltext  = leg.get_texts()
plt.setp(ltext,family='CMUSerif-Roman',fontsize=16)

# Adjust the page layout filling the page using the new tight_layout command
plt.tight_layout(pad=0.5)
# plt.savefig('/Users/gerald/Documents/Figures/PID_MRAC_Reference.pdf',transparent=True)
# plt.savefig('/Users/gerald/Documents/Figures/PID_MRAC_Plant_no_adjustment.pdf',transparent=True)
# plt.savefig('/Users/gerald/Documents/Figures/PID_MRAC_Plant_adjustment.pdf',transparent=True)
# plt.savefig('/Users/gerald/Documents/Figures/PID_MRAC_Realplant_adjustment.pdf',transparent=True)
# plt.savefig('/Users/gerald/Documents/Figures/PID_MRAC_Realplant_no_adjustment.pdf',transparent=True)
# plt.savefig('/Users/gerald/Documents/Figures/PID_MRAC_Plant_adjustment_unstable.pdf',transparent=True)




plt.show()





