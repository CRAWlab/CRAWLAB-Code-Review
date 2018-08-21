
#!/usr/bin/env python
#########################################################################################
#
#
# File name: husky_odom_PD.py
#
# Input waypoints and have husky turn to face and move toward input points
#
# Created: 01/26/18
#	- Jacob Randall
#	- jacobtrueno86@gmail.com
#
# Modified: 
#	04/15/18
#
# TODO:
#	- 
# 
#########################################################################################

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from husky_PID import PID
import angles
import time as ti
import matplotlib.pyplot as plt

# Set values for the different gains.
kp_move = 2.95283743654
ki_move = 0.0
kd_move = 0#4.39799357199 

kp_turn = 2.0
ki_turn = 0.0
kd_turn = 1.14789471589

class draw_a_square(object):

	def __init__(self):

		# Call the angles script
		self.ang = angles

		# Set a threshold for the velocity to fall under before stopping
		self.threshold = 0.005

		# Define the side length
		self.length = 5

		# Define the angle to turn in degrees converted to radians
		self.turn_angle = np.radians(90)

		# Create a command to move forward
		self.move_cmd = Twist()

		# Define the turning rate in rad/s 
		self.turn_cmd = Twist()
		
		# Define the publisher to publish commands
		self.p = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		rospy.sleep(0.10)

		# Define the listener from the tf library to subscribe to odom topic
		self.listener = tf.TransformListener()
		rospy.sleep(0.10)
	
	def odom_lookup(self):

		# Try to look up the position odometry information
		try:
			((x,y,z), rot) = self.listener.lookupTransform('odom', 'base_link', rospy.Time(0))
			
		# Raise exceptions if it doesn't work
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print 'Position didnt work'
			raise

		# Transforms the orientation from quaternion to euler so we can use data
		(phi, psi, theta) = tf.transformations.euler_from_quaternion(rot)
		
		# Look up the velocity odometry information
		try:
			((v_x, v_y, v_z), (w_x, w_y, w_z)) = self.listener.lookupTwist('odom', 'base_link', rospy.Time(0), rospy.Duration(0.01))

		# Raise exceptions if it doesn't work
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print 'Velocity didnt work'
			raise

		# Calculate resultant velocity from x and y vectors
		self.velocity = np.sqrt(v_x**2 + v_y**2)
		
		# Set omega equal to the rotational speed around the z axis
		self.omega = w_z

		return x, y, theta, self.velocity, self.omega


	def move(self, x, y, kp, kd, ki):

		# Call the PID class from husky_PID
		self.pid = PID(kp, ki, kd, 0.15, 100, -100, 0.0)

		# Define starting values 
		x_start = x
		y_start = y
		self.distance = 0
		self.last_state = 0
		self.max_distance = 0
		i = 1
		
		# Create a list to collect data
		position = []
		time = []

		# Make the starting values for time and position 0
		time.append(0)
		position.append(0)

		# Define a starting time
		t_start = ti.time()

		# Delay 0.5 seconds for graph clarity
		ti.sleep(0.5)

		# While my distance is less than my length and my velocity is greater than desired keep moving
		while abs(self.length - self.distance) >= self.threshold or self.velocity >= self.threshold :
			
			# Move forward at the speed defined above
			self.p.publish(self.move_cmd)
			rospy.loginfo('moving forward')
		
			# Get the latest odometry information
			x, y, theta, self.velocity, self.omega = self.odom_lookup()
			
			# Calculate the distance traveled
			self.distance = np.sqrt(pow(x - x_start, 2) + pow(y - y_start, 2))
		
			# Compute output from husky_PID script
			self.output = self.pid.compute_output(self.last_state, self.length, self.distance, None, None)
			
			# Update the last state for husky_PID script to properly calculate state_change
			self.last_state = self.distance
			
			# Set the published command to the calculated output
			self.move_cmd.linear.x = self.output 

			# Get max distance for overshoot used in twiddle
			if self.distance > self.max_distance:
				self.max_distance = self.distance

			# Get overshoot for cost function in twiddle
			self.overshoot = self.max_distance - self.length

			# Calculate the current elapsed time
			t = ti.time() - t_start

			# Add the current elapsed time and position to the lists of data
			time.append(t)
			position.append(self.distance)

			# If it takes longer than estimated to complete action then terminate
			i = i + 1
			if i == 5000:
				break
			print i

		# Add data until 10 seconds for graphing purposes
		while t < 10:

			t = ti.time() - t_start

			time.append(t)
			position.append(self.distance)
		
		return x, y, theta, self.distance, self.overshoot, time, position
		

	def turn(self, theta, kp, kd, ki):

		# Call the PID class from husky_PID
		self.pid = PID(kp, ki, kd, 0.15, 100, -100, 0.0)

		# Create a list to collect data
		position = []
		time = []

		# Make the starting values for time and position 0
		time.append(0)
		position.append(0)

		# Define a starting time
		t_start = ti.time()

		# Define starting angle
		self.angle_start = theta
		
		# Define starting value to be passed into compute_output function
		self.last_state = 0

		self.overshoot = 0

		# Calculate the shortest angle to the goal
		self.shortest_turn = self.ang.shortest_angular_distance(theta, self.angle_start + self.turn_angle)

		# While the shortest turn and the angular velocity are greater than the threshold keep turning
		while abs(self.shortest_turn) >= self.threshold or abs(self.omega) >= self.threshold:

			# Turn at the speed defined
			self.p.publish(self.turn_cmd)
			rospy.loginfo('turning')

			# Recalculate the shortest angle to goal
			self.shortest_turn = self.ang.shortest_angular_distance(theta, self.angle_start + np.radians(360) + self.turn_angle)

			# Current state defined to pass into compute_output
			self.current_state = self.turn_angle - self.shortest_turn

			# Calculate the desired rate of turning
			self.output = self.pid.compute_output(self.last_state, self.turn_angle, self.current_state, None, None)

			# Update the last state
			self.last_state = self.current_state

			# Set the published comman to the calculated output
			self.turn_cmd.angular.z = self.output

			# Look up the latest and greatest odometry information
			x, y, theta, self.velocity, self.omega = self.odom_lookup()

			self.angle_turned = abs(self.ang.shortest_angular_distance(self.angle_start, theta))

			self.turn_error = self.angle_turned - abs(self.turn_angle)

			if self.turn_error > self.overshoot:
				self.overshoot = self.turn_error

			# Calculate the current elapsed time
			t = ti.time() - t_start

			# Add the current elapsed time and position to the lists of data
			time.append(t)

		return x, y, theta, self.overshoot, time


if __name__ == '__main__':

	# Initialize the node
	rospy.init_node('odom_sub')

	# Call the class
	das = draw_a_square()

	try:

		x, y, theta, velocity, omega = das.odom_lookup()
		
		x, y, theta, distance, overshoot, time, position = das.move(x, y, kp_move, kd_move, ki_move)
		
		'''x, y, theta, overshoot, time = das.turn(theta, kp_turn, kd_turn, ki_turn)

		x, y, theta, distance, overshoot, time, position = das.move(x, y, kp_move, kd_move, ki_move)

		x, y, theta, overshoot, time = das.turn(theta, kp_turn, kd_turn, ki_turn)

		x, y, theta, distance, overshoot, time, position = das.move(x, y, kp_move, kd_move, ki_move)

		x, y, theta, overshoot, time = das.turn(theta, kp_turn, kd_turn, ki_turn)

		x, y, theta, distance, overshoot, time, position = das.move(x, y, kp_move, kd_move, ki_move)

		x, y, theta, overshoot, time = das.turn(theta, kp_turn, kd_turn, ki_turn)'''
		
		# Make the step input
		init = 0.0
		desired_output = np.ones_like(time)*init
		desired_output[3:] = das.length
	

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
		ax.grid(True,linestyle=':', color='0.75' )
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
		plt.savefig('P_low_Control.svg', transparent = True)

		# show the figure
		plt.show()
		
	except:

		print 'node terminated'	
		raise
