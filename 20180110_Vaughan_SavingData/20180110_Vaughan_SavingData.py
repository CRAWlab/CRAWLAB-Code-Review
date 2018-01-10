#! /usr/bin/env python

###############################################################################
# 20170110_Vaughan_SavingData.py
#
# script demonstrating how to save data to a csv file with a header
#
#
# Created: 01/10/18
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

import numpy as np
import datetime         # We'll use this to define a unique filename


# First we'll generate some data to save
t = np.linspace(0, 5, 501)         # 0-5s with 501 points
data1 = np.sin(t)                  # 1 rad/s sine wave
data2 = np.sin(2 * np.pi * t)      # 1Hz sine wave
data3 = np.sin(5 * (2*np.pi) * t)  # 5Hz sine wave

# Now, we should form the vector of data
# We want the each "piece" of the data to be column, so we need to shape it 
# accordingly. Here, we're using the NumPy stack method:
#   https://docs.scipy.org/doc/numpy-1.13.0/reference/generated/numpy.stack.html#numpy.stack
data_to_save = np.stack((t, data1, data2, data3), axis=1)

# For experimental results, we often want to identify the trial using a timestamp, 
# we can do that by 
data_filename = 'Data_' + datetime.datetime.now().strftime('%Y-%m-%d-%H%M%S') + '.csv'
    
# Note that it's probably a good idea to include more info in the filename and
# to name it more distinctly than Data. You can/should use string formatting 
# methods to do this automatically:
#   https://pyformat.info
#
# The new, in Python 3.6, f-strings are nice too. This is a good overview:
#  https://www.blog.pythonlibrary.org/2017/02/08/new-in-python-formatted-string-literals/

# Now, let's save this data
#
# First, we'll create the string defining the header
# Be sure to name your columns to represent the actual data and include units
file_header = 'Time (s), Column 1 (units), Data Column 2 (units), Column 3 (units)'
    
# Then, we save the file
np.savetxt(data_filename, data_to_save, header=file_header, delimiter=',')