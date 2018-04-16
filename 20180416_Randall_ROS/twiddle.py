# usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from husky_PID import PID
from husky_odom_PD import draw_a_square

if __name__ == '__main__':

	rospy.init_node('twiddle')

	# Call the class with the function I am tuning for
	das = draw_a_square()
	kp = 1.0
	kd = 1.0

	# Set importance for parameters of cost function
	a = 1
	b = 1

	# Create a parameter and change in parameter array
	p = np.array([kp, kd])
	dp = np.array([1.0, 1.0])

	# Get odometry informtation to use as inputs for function
	x, y, theta, velocity, omega = das.odom_lookup()

	# Run my function I am tuning for
	x, y, theta, overshoot, time = das.turn(theta, p[0], p[1])
	
	# Get the settling time
	settling_time = time[-1]

	# Create the cost function
	cost = a*overshoot**2 + b*settling_time**2

	# Set my best error
	best_error = cost

	while np.sum(dp) > .1:

		# Do this for each parameter
		for i in range(len(p)):

			print "\n"
			print p
			print dp
			print"\n"

			# Add the change in parameter
			p[i] += dp[i]

			# Run the function
			x, y, theta, max_turn, time = das.turn(theta, p[0], p[1])

			settling_time = time[-1]
			cost = a*overshoot**2 + b*settling_time**2

			# Define the error
			error = cost
			print "\n"
			print best_error
			print error
			print "percent overshoot", overshoot, "settling_time", settling_time
			
			
			if error < best_error:

				print'\n There was improvement \n'
				# Make this error my new best error
				best_error = error
				# Increase the change in parameter
				dp[i] *= 1.1

			else:

				print'\n There was no improvement \n'

				# Go in the other direction for that parameter
				p[i] -= 2*dp[i]

				x, y, theta, overshoot, time = das.turn(theta, p[0], p[1])

				settling_time = time[-1]
				cost = a*overshoot**2 + b*settling_time**2

				error = cost
				print "\n"
				print best_error
				print error
				print "percent overshoot", overshoot, "settling_time", settling_time

				if error < best_error:

					print '\n There was improvement \n'

					best_error = error
					dp[i] *= 1.1

				else:

					print'\n There was no improvement \n'

					p[i] += dp[i]
					dp[i] *= 0.9

	print "kp", p[0]
	print "kd", p[1]