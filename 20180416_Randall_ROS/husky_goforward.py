
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from husky_odom_PD import draw_a_square
import matplotlib.pyplot as plt
import time as ti
import numpy as np
import tf

# Set the length of movement
length = 5

# Create publish command
p = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
rospy.sleep(0.05)

# Define the listener from the tf library to subscribe to odom topic
listener = tf.TransformListener()
rospy.sleep(0.10)

# Set the velocity command
move_cmd = Twist()
move_cmd.linear.x = 1.0

def odom_lookup():

		# Try to look up the position odometry information
		try:
			((x,y,z), rot) = listener.lookupTransform('odom', 'base_link', rospy.Time(0))
			
		# Raise exceptions if it doesn't work
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print 'Position didnt work'
			raise

		# Transforms the orientation from quaternion to euler so we can use data
		(phi, psi, theta) = tf.transformations.euler_from_quaternion(rot)
		
		# Look up the velocity odometry information
		try:
			((v_x, v_y, v_z), (w_x, w_y, w_z)) = listener.lookupTwist('odom', 'base_link', rospy.Time(0), rospy.Duration(0.01))

		# Raise exceptions if it doesn't work
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print 'Velocity didnt work'
			raise

		# Calculate resultant velocity from x and y vectors
		velocity = np.sqrt(v_x**2 + v_y**2)
		
		# Set omega equal to the rotational speed around the z axis
		omega = w_z

		return x, y, theta, velocity, omega

def move():

	# Publish velocity command
	p.publish(move_cmd)
	rospy.loginfo('moving forward')

if __name__ == '__main__':

	# Initialize the node
	rospy.init_node('goforward')  

	# Create empty lists to append to as data is collected
	position = []
	time = []

	# Set the starting values to 0
	time.append(0)
	position.append(0)

	# Define a start time 
	t_start = ti.time()

	# Set starting values
	distance = 0
	x_start = 0
	y_start = 0
	
	# Create true statements for conditionals that happen once
	yes = True
	cool = True

	try:

		while distance <= length:

			# Lookup the odometry information
			x, y, theta, velocity, omega = odom_lookup()

			# Set starting x and y values
			if yes:
				x_start = x
				y_start = y
				yes = False

			# Calculate the distance and elapsed time
			distance = np.sqrt((x-x_start)**2 + (y-y_start)**2)
			t = ti.time() - t_start

			# Add the elapsed time and distance to lists
			time.append(t)
			position.append(distance)

			# Delay everything for 0.5 seconds for graph clarity
			if cool:
				ti.sleep(0.5)
				cool = False

			move()

		# Add data that goes to 10 seconds for graphing purposes
		while t < 10:

			x, y, theta, velocity, omega = odom_lookup()

			distance = np.sqrt((x-x_start)**2 + (y-y_start)**2)
			t = ti.time() - t_start

			time.append(t)
			position.append(distance)

	except:
		print 'you suck'
		raise

	print time[-1]

	# Create the step command
	init = 0.0
	desired_output = np.ones_like(time)*init
	desired_output[3:] = length
	

	# Set the plot size - 3x2 aspect ratio is best
	fig = plt.figure(figsize=(6,4))
	ax = plt.gca()
	plt.subplots_adjust(bottom=0.17, left=0.17, top=0.96, right=0.96)

	# Change the axis units font
	plt.setp(ax.get_ymajorticklabels(),fontsize=18)
	plt.setp(ax.get_xmajorticklabels(),fontsize=18)

	ax.spines['right'].set_color('none')
	ax.spines['top'].set_color('none')

	ax.xaxis.set_ticks_position('bottom')
	ax.yaxis.set_ticks_position('left')

	# Turn on the plot grid and set appropriate linestyle and color
	ax.grid(True,linestyle=':', color='0.75')
	ax.set_axisbelow(True)

	# Define the X and Y axis labels
	plt.xlabel('Time (s)', fontsize=22, weight='bold', labelpad=5)
	plt.ylabel('Position (m)', fontsize=22, weight='bold', labelpad=10)
 
 	plt.plot(time, desired_output, linewidth=2, linestyle='--', label=r'Command', color = '#e41a1c') 
	plt.plot(time, position, linewidth=2, linestyle='-', label=r'Response', color = '#377eb8')

	# uncomment below and set limits if needed
	plt.xlim(0,10)
	plt.ylim(0,8)

	# Create the legend, then fix the fontsize
	leg = plt.legend(loc='upper right', ncol = 1, fancybox=True)
	ltext  = leg.get_texts()
	plt.setp(ltext,fontsize=18)

	# Adjust the page layout filling the page using the new tight_layout command
	plt.tight_layout(pad=0.5)

	# save the figure as a high-res pdf in the current folder
	plt.savefig('No_Control.svg', transparent = True)

	# show the figure
	plt.show()