#! /usr/bin/env python

###############################################################################
# 20180205_Vaughan_Exceptions.py
#
# This is script demonstrating the basics of handling exceptions in Python
#
# NOTE: Any plotting is set up for output, not viewing on screen.
#       So, it will likely be ugly on screen. The saved PDFs should look
#       better.
#
# Created: 02/04/18
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * 
#
# TODO:
#   * 
###############################################################################

import time 

# In this example we'll create an exception by attempting to divide by zero. 
# This will create a ZeroDivisionError for that operation. 

# We get this error anytime we try to divide by 0. If you run this script without the line 
# below commented out, you should get a traceback to this line of the script and an 
# explanation of the error
# test_exception = 1/0

# If we instead, wrap our calculation in a try... except block looking for this error
# we can "catch" it and prevent the entire script from crashing (if we want)
try:
    test_exception = 1/0
    print('The result is: {:0.3f}'.format(test_exception))

except(ZeroDivisionError):
    print('Error: You are trying to divide by zero.')
    
    # If you do want the script to crash even when catching the exception and running code
    # you want to run, you can then call raise to re-raise the exception you caught. 
    # Uncomment the line below to see this in action.
    # raise
    

# We can also add a finally to created a try... except... finally block. The code within
# the finally block will *always* be run, whether and exception occurred or not
try:
    test_exception = 1/0
    print('The result is: {:0.3f}'.format(test_exception))

except(ZeroDivisionError):
    print('Error: You are trying to divide by zero.')

finally:
    print('Code in the finally block will *always* run.')


# A common construction for us will have a `while True:` statement containing the system's
# control loop inside a `try... except... finally` block. We will most often 
# explicitly catch the KeyboardInterrupt and SysExit exceptions. The KeyboardInterrupt
# exception is what is called when we stop a script using Control-C from the terminal.

try:
     while True:
        print('Running the control loop...')
        time.sleep(0.1)
        
except (KeyboardInterrupt, SystemExit):
    # In the except block, we will typically only catch KeyboardInterrupt. 
    # Whatever we do in this block needs to happen quickly relative to the system 
    # dynamics. If there is an exception it will run prior to the finally clause. 
    # So, if we only have the safe-stop code in the finally block, it'll be waiting for 
    # this clause ot finish running. One option to combat this is to include the code
    # necessary to stop the system safely both here and in the finally block. The downside
    # to this is that the code is repeated (usually bad practice) and, in some cases, may
    # cause addition errors as the code will be attempting to close files, servers, etc.
    # and stop motors, etc more than once.
    print('In some cases, we should stop the system here.')
    
    # We'll often save the latest data here, etc. 
    print('Typically, we will save data, etc. in the except clause.')

finally:
    print('Then, safely stop the machine/robot by stopping motors, etc.')