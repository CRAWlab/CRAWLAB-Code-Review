# Constrained Optimization Using scipy minimize
## Finding optimal trajectories with arbitrary initial and final states


Finds a trajectory for a planar system with two masses

The degree of freedom in the x-direction is shown here
```python
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
```


Begin by importing the necessary libraries
```python
import numpy as np
from scipy.integrate import odeint

#needed to perform the minimization
from scipy.optimize import minimize

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches
```
Define constants to be used throughout the scrpit
```python
# Constant parameters for equations of motion
# controls the error in the simulated response
ABSERR = 1.0e-9
RELERR = 1.0e-9
# maximum absolute step size allowed
MAX_STEP = 0.01
# Stepsize of time array to return results
DELTA_T = 0.01
```

Next define the equations of motion in state space-ish form
```python
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
  # tm – list of times at which each element of the command is initiated

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
```

This accepts as an argument a list of forces in the x and y directions. To determine what force to apply at each time
```python
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
```

The cost function aims to minimize the trajectory length of the rigid mode
```python
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
```

Define the constraints used for the minimization problem.

For minimization, constraints are written as:
$h_{i}(x)=0$
$g_{i}(x)\leq 0$

The python function containing the constraint functions consists of a tuple of dictionaries for the constraints.

```python
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
```

Finally, the function that runs the optimization is given below
```python
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
```




