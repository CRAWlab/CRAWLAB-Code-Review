#! /usr/bin/env python

##########################################################################################
# Trajectory_gen.py
#
# Generates an optimal trajectory with arbitrary initial and final states
#
#
# Created: 3/23/18
#
##########################################################################################



# 
#                  |--->x1                    |--->x2
#                  |                          |
#           +------------+      k       +------------+
#           |            |-\/\/\/\/\/\/-|            |
# u(t)----->|     m1     |              |     m2     |
#           |            |-----||-------|            |
#           +------------+      c       +------------+
# 
# 




import numpy as np
from scipy.integrate import odeint
from scipy.optimize import minimize

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches

# Constant parameters for equations of motion
# controls the error in the simulated response
ABSERR = 1.0e-9
RELERR = 1.0e-9
# maximum absolute step size allowed
MAX_STEP = 0.001
# Stepsize of time array to return results
DELTA_T = 0.001


def eq_of_motion(w, tsim, p):
  """Define the equation of motion in state space-ish form"""

	# unpack the states of the system
  x1, x1_dot, x2, x2_dot, y1, y1_dot, y2, y2_dot = w
  # unpack parameters for the simulation
  m1, m2, k, c, ux, uy, tm = p

  # m1 – mass of the rigid mode
  # m2 – mass of the flexible mode
  # k – stiffness
  # c – damping coefficient
  # ux – list of force input in the x-direction
  # uy – list force input in the y-direction
  # tm – list of times at which each element in the command list is initiated

  sysODE = [x1_dot,
            (1/m1)*(command(ux,tm,tsim)+k*(x2-x1)+c*(x2_dot-x1_dot)),
            x2_dot,
            (1/m2)*(-k*(x2-x1)-c*(x2_dot-x1_dot)),
            y1_dot,
            (1/m1)*(command(uy,tm,tsim)+k*(y2-y1)+c*(y2_dot-y1_dot)),
            y2_dot,
            (1/m2)*(-k*(y2-y1)-c*(y2_dot-y1_dot)),
            ]
    
  return sysODE


def command(u,tm,t):
  """The command to be passed to the equations of motion 
  u - list of commands used for path 
  tm - list of times at which each command starts
  t - simulation time"""

  num_com=len(u)
  for i in range(num_com):
    if t >= tm[i] and t < tm[i+1]:
      com=u[i]
      # if t is between tm[i] and tm[i+1], use u[i] as the command input
      break
    else:
      com=0

  return com


class Node:
  """Class containing the attributes of each node in the Rapid-exploring Random Tree"""
  x = 0 # position of the rigid mode
  y = 0
  xf = 0 # position of the flexible mode
  yf = 0

  x_dot=0 # velocity of the rigid mode
  y_dot=0
  xf_dot=0 # velocity of the flexible mode
  yf_dot=0

  def __init__(self,xcoord, ycoord):
       self.x = xcoord
       self.y = ycoord



# function to run odeint to simulate the response of the system
def response(x,args):
  """Run odeint to evaluate the equations of motion and simulate the response"""

  # unpack the arguments
  initial_node, final_node, m1, m2, k, c, vmax, umax, command_length = args

  ux=x[0:command_length] # list of force in x-direction
  uy=x[command_length:2*command_length] # list of force in y-direction
  tm=np.linspace(0,x[-1],command_length+1) # time array corresponding to command initiation
  

  t=np.arange(0,x[-1],DELTA_T) # time array for simulation

  # initial condition for simulation
  x0=[initial_node.x,initial_node.x_dot,initial_node.xf,initial_node.xf_dot,initial_node.y,initial_node.y_dot,initial_node.yf,initial_node.yf_dot]

  param=m1, m2, k, c, ux, uy, tm


  resp=odeint(eq_of_motion, x0, t, args=(param,), atol = ABSERR, rtol = RELERR, hmax=MAX_STEP)

  return resp




def func(x,args):
  """Calculates the distance of the response
      Minimize the distance"""

  # unpack args
  initial_node, final_node, m1, m2, k, c, vmax, umax, command_length = args


  resp = response(x,args)
  # Calculate the distance traveled from node to goal
  xp=np.asarray(resp[:,0])
  yp=np.asarray(resp[:,4])

  di=np.sqrt(np.diff(xp)**2+np.diff(yp)**2) # distance between each point on path
  path_length=np.sum(di) # The actual length of the trajectory

  return path_length



def form_constraints(x,args):
  """Define the constraints to be used for optimization"""

  # unpack args
  initial_node, final_node, m1, m2, k, c, vmax, umax, command_length = args


  # make a tuple of dictionaries to define constraints
  const=({'type': 'ineq', 'fun':lambda x: umax-abs(x[0:-1])}, # absolute value input must be less than maximum allowable input

    {'type': 'eq', 'fun':lambda x: response(x,args)[-1,0]-final_node.x}, # last coordinate of trajectory must reach goal
    {'type': 'eq', 'fun':lambda x: response(x,args)[-1,4]-final_node.y},

    # {'type': 'eq', 'fun':lambda x: response(x,args)[-1,1]-final_node.x_dot},
    # {'type': 'eq', 'fun':lambda x: response(x,args)[-1,5]-final_node.y_dot},

    )

  return const



def plan_traj(x0,args):
  """Find the optimal trajectory to connect the tree to the goal"""

  const=form_constraints(x0,args)


  res = minimize(func, x0, args,
    constraints = const, method='SLSQP', 
    options={'eps':1e-3, 'disp':True, 'ftol':1e-3,'maxiter':1000})

  if res.success:
    print('Found an optimal trajectory.')
  else:
    print('An optimal trajectory could not be found')

  return res.x


# if the script is run directly, run the following code
if __name__ == '__main__':

  # system parameters
  m1=5     # kg
  m2=1     # kg
  k=1      # N/m
  c=0      # N-s/m
  vmax=5   # m/s
  umax=10  # N


  # initial position
  x_init=5
  y_init=5

  # initial velocity
  xdot_init=5
  ydot_init=2

  # final position
  x_final=15
  y_final=20

  # final velocity
  xdot_final=0
  ydot_final=0

  # initiate node with these attributes
  initial_node = Node(x_init,y_init)
  initial_node.x_dot=xdot_init
  initial_node.y_dot=ydot_init

  # final node with these attributes
  final_node = Node(x_final,y_final)
  final_node.x_dot=xdot_final
  final_node.y_dot=ydot_final


  # initial guess for optimization
  command_length=15 # length of the command in x and y-directions
  init_time=1 # nonzero initial guess of command duration
  x0=np.zeros(2*command_length+1)
  x0[-1]=init_time

  args = [initial_node, final_node, m1, m2, k, c, vmax, umax, command_length]

  # find the minimum length trajectory that has the given initial and final states
  res = plan_traj(x0,args)


  # simulate the response using the solution to the optimization
  resp = response(res,args)


  # np.save('Trajectory_gen_resp',resp)


  fig = plt.figure(figsize=(6,4))
  ax = plt.gca()
  plt.subplots_adjust(bottom=0.17,left=0.17,top=0.96,right=0.96)
  plt.setp(ax.get_ymajorticklabels(),family='serif',fontsize=18)
  plt.setp(ax.get_xmajorticklabels(),family='serif',fontsize=18)
  ax.spines['right'].set_color('none')
  ax.spines['top'].set_color('none')
  ax.xaxis.set_ticks_position('bottom')
  ax.yaxis.set_ticks_position('left')
  ax.grid(True,linestyle=':',color='0.75')
  ax.set_axisbelow(True)


  plt.xlim(0,25)
  plt.ylim(0,25)

  plt.xlabel('X Position',family='CMUSerif-Roman',fontsize=22,weight='bold',labelpad=5)
  plt.ylabel('Y Position',family='CMUSerif-Roman',fontsize=22,weight='bold',labelpad=10)

  # plt.plot(resp[:,0],resp[:,4],linewidth=2,linestyle = '-', label='Trajectory')
  responseLine, = plt.plot([],[],linewidth=2,linestyle = '-', label='Trajectory')

  response_shape = mpatches.Circle(([],[]),0.1,ec='none',fc='blue')

  # leg = plt.legend(loc='upper left', fancybox=True)
  # ltext  = leg.get_texts()
  # plt.setp(ltext,family='serif',fontsize=16)

  plt.tight_layout(pad=0.5)
  # plt.savefig('/Users/gerald/Documents/Figures/CrawLab Spring 2018 presentation/Trajectory Gen.pdf',transparent=True)



  # The initial state of the history line
  # resp[:,0] is response of the rigid mode in the x direction
  # resp[:,4] response of rigid mode in the y direction
  def init():
      responseLine.set_data([],[])
      response_shape.center = (resp[0,0],resp[0,4])
      ax.add_patch(response_shape)
      return responseLine, response_shape



  def animate(i):
      print('Be patient. Only {}'.format(len(resp)-i), 'frames to go.')
      response_shape.center = (resp[i,0],resp[i,4])
      responseLine.set_data(resp[0:i,0],resp[0:i,4])

      return responseLine, response_shape

  ani = animation.FuncAnimation(fig, animate, frames=len(resp), init_func=init)

  # # save the animation as an mp4.  This requires ffmpeg or mencoder to be
  # # installed.  The extra_args ensure that the x264 codec is used, so that
  # # the video can be embedded in html5.  You may need to adjust this for
  # # your system: for more information, see
  # # http://matplotlib.sourceforge.net/api/animation_api.html
  ani.save('Trajectory_gen4.mp4', bitrate = 1500, fps=1000*res[-1])








