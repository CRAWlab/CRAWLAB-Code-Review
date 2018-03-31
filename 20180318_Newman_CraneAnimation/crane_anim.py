"""
#------------------------------------------------------------------------------
# crane_anim.py
#
# This script imports previously-saved simulation data and creates an animation
# for showing on a powerpoint or Keynote presentation 
#
# Created: 3/18/18 - Daniel Newman -- danielnewman09@gmail.com
#
# Modified:
#   * 3/18/18 - DMN -- danielnewman09@gmail.com
#       - Added documentation for this script
#   * 03/31/18 - JEV - joshua.vaughan@louisiana.edu
#       - Added alternate encoding options, perhaps better as "standalone" video
#------------------------------------------------------------------------------
"""
import numpy as np
import matplotlib.pylab as plt
import scipy
from scipy import ndimage
import matplotlib.patheffects as path_effects
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D
from matplotlib.animation import FuncAnimation, writers

# Maintain consistency between different plot colors
UNSHAPED_COLOR = '#e41a1c'
SHAPED_COLOR = '#377eb8'

# Determine whether we want to make a transparent background 
# for clearner presentation
OPAQUE = False
TRANSPARENT = not OPAQUE

# These are some constants for my plotting
# I have multiplicative factors to give decent room above 
# and below the plotted data, making my figures look cleaner
Y_MIN = 0.1
Y_MAX = 0.3

# Locate the center of the trolley image at these coordinates
TROLLEY_OFFSET_X = 100
TROLLEY_OFFSET_Y = 100

# Because we are importing an external image, we need to scale the
# response values to match the size of the image
SCALING_FACTOR = 500

# Load an external image for the trolley
TROLLEY_IMAGE = plt.imread("Trolley.png")

# Specify axis labels
X_LABEL = 'Time (s)'
Y1_LABEL = 'Trolley Velocity (m/s)'
Y2_LABEL = 'Payload Angle (deg)'

# Let's load response data from a previously run simulation
time = np.load('time.npy')

# Framerate of the animation, derived from the sampling time 
# of the simulation
fps = int(np.round(1 / (time[1] - time[0])))

# Load the responses saved from the previous simulation
unshaped_response = np.load('unshaped_response.npy')
shaped_response = np.load('shaped_response.npy')

# extract the relevant response data for our animation
unshaped_trolley_x = unshaped_response[:,2] * SCALING_FACTOR
shaped_trolley_x = shaped_response[:,2] * SCALING_FACTOR
unshaped_theta = unshaped_response[:,0]
shaped_theta = shaped_response[:,0]

# Cable length of the crane 
length = SCALING_FACTOR

# Create a new figure large enough to fill a powerpoint screen
fig = plt.figure(figsize=(16,9))

# Create a 2x2 subplot
gs1 = gridspec.GridSpec(2, 2)

# Set the grid spacing as appropriate
gs1.update(left=0, right=0.95, hspace=0.2, wspace=0.1)

# Set the axes for the subplots. 
ax1 = plt.subplot(gs1[:, :-1])
ax2 = plt.subplot(gs1[0, -1])
ax3 = plt.subplot(gs1[-1, -1])

# Create a line for the unshaped velocity plot
unshaped_velocity, = ax2.plot(
                        [], [], # Set the data to null at first
                        lw=2, # Line width
                        path_effects=[path_effects.SimpleLineShadow(),path_effects.Normal()], # Add a shadow
                        color=UNSHAPED_COLOR, # Set the color
                        linestyle='-') # Simple linestyle

# Create a line for the unshaped theta plot
unshaped_theta_line, = ax3.plot(
                        [],[], # Set the data to null at first
                        lw=2, # Line width
                        path_effects=[path_effects.SimpleLineShadow(),path_effects.Normal()], # Add a shadow
                        color=UNSHAPED_COLOR, # Set the color
                        linestyle='-') # Simple linestyle

# Create a line for the shaped velocity plot
shaped_velocity, = ax2.plot(
                        [], [], # Set the data to null at first
                        lw=2, # Line width
                        path_effects=[path_effects.SimpleLineShadow(),path_effects.Normal()], # Add a shadow
                        color=SHAPED_COLOR, # Set the color
                        linestyle='--') # Simple linestyle

# Create a line for the shaped theta plot
shaped_theta_line, = ax3.plot(
                        [], [], # Set the data to null at first
                        lw=2, # Line width
                        path_effects=[path_effects.SimpleLineShadow(),path_effects.Normal()], # Add a shadow
                        color=SHAPED_COLOR, # Set the color
                        linestyle='--') # Simple linestyle

# Create null lists for all the data
x_data = []
unshaped_veldata = []
unshaped_thetadata  = []
shaped_veldata  = []
shaped_thetadata = []

# Set the X limits for the velocity and theta plots
ax2.set_xlim(np.amin(time), np.amax(time))
ax3.set_xlim(np.amin(time), np.amax(time))

# Set the Y limits for the velocity and theta plots based on the constants already defined
ax2.set_ylim(
        np.amin(unshaped_response[:,3]) - Y_MIN * abs(np.amin(unshaped_response[:,3])),
        np.amax(unshaped_response[:,3]) + Y_MAX * abs(np.amax(unshaped_response[:,3])-np.amin(unshaped_response[:,3])))
ax3.set_ylim(
        np.amin(np.rad2deg(unshaped_response[:,0])) - Y_MIN * abs(np.rad2deg(np.amin(unshaped_response[:,0]))),
        np.amax(np.rad2deg(unshaped_response[:,0])) + Y_MAX * abs(np.amax(np.rad2deg(unshaped_response[:,0]))-np.amin(np.rad2deg(unshaped_response[:,0]))))

# Set the plot window and labels for the first plot
ax2.spines['right'].set_color('none')
ax2.spines['top'].set_color('none')
ax2.xaxis.set_ticks_position('bottom')
ax2.yaxis.set_ticks_position('left')
ax2.grid(False)
ax2.set_xlabel('{}'.format(X_LABEL), fontsize=24, weight='bold', labelpad=5)
ax2.set_ylabel('{}'.format(Y1_LABEL), fontsize=24, weight='bold', labelpad=5)

# Set the plot window and labels for the first plot
ax3.spines['right'].set_color('none')
ax3.spines['top'].set_color('none')
ax3.xaxis.set_ticks_position('bottom')
ax3.yaxis.set_ticks_position('left')
ax3.grid(False)
ax3.set_xlabel('{}'.format(X_LABEL), fontsize=24, weight='bold', labelpad=5)
ax3.set_ylabel('{}'.format(Y2_LABEL), fontsize=24, weight='bold', labelpad=5)

# Set the X and Y endpoints for the unshaped cable
unshaped_cable_end_x = unshaped_trolley_x - length * np.sin(unshaped_theta)
unshaped_cable_end_y = - length * np.cos(unshaped_theta)

# Set the X and Y endpoints for the shaped cable
shaped_cable_end_x = shaped_trolley_x - length * np.sin(shaped_theta)
shaped_cable_end_y = - length * np.cos(shaped_theta)

def update(i):

    # Since we're using loaded images, we need to clear the axis
    # on which we display the images
    ax1.cla()

    if not (i % fps): # print notice every 30th frame
        print('Processing frame {}'.format(i))

    # Add the next data point to the plot lines
    x_data.append(time[i])
    unshaped_veldata.append(unshaped_response[i,3])
    unshaped_thetadata.append(unshaped_response[i,0])
    shaped_veldata.append(shaped_response[i,3])
    shaped_thetadata.append(shaped_response[i,0])

    # Set the plot lines with the up-to-date data
    unshaped_velocity.set_data(x_data, unshaped_veldata) 
    unshaped_theta_line.set_data(x_data,np.rad2deg(unshaped_thetadata)) 
    shaped_velocity.set_data(x_data, shaped_veldata) 
    shaped_theta_line.set_data(x_data,np.rad2deg(shaped_thetadata))   

    # Hide grid lines on the trolley window
    ax1.grid(False)

    ax1.set_ylim(-600,150)
    ax1.set_xlim(-300,1000)

    # Hide the window
    ax1.spines['right'].set_color('none')
    ax1.spines['top'].set_color('none')
    ax1.spines['bottom'].set_color('none')
    ax1.spines['left'].set_color('none')

    # Hide axes ticks
    ax1.set_xticks([])
    ax1.set_yticks([])

    # Add the line representing the unshaped cable motion
    ax1.add_line(
            Line2D(
                [unshaped_trolley_x[i],unshaped_cable_end_x[i]], # Start and end X coordinates
                [0,unshaped_cable_end_y[i]], # Start and end Y coordinates
                linewidth=5, # Line weight
                linestyle='-', # Simple line style
                marker='o', # Add circles at the endpoints
                label='Bar', 
                color='#e41a1c', # Set the color to be consistent for unshaped motion
                path_effects=[path_effects.SimpleLineShadow(),path_effects.Normal()]) # Add a shadow
                )

    # Add the line representing the shaped cable motion
    ax1.add_line(
            Line2D(
                [shaped_trolley_x[i],shaped_cable_end_x[i]], # Start and end X coordinates
                [0,shaped_cable_end_y[i]], # Start and end Y coordinates
                linewidth=5, # Line weight
                linestyle='-', # Simple line style
                marker='o', # Add circles at the endpoints
                label='Bar',
                color='#377eb8', # Set the color to be consistent for unshaped motion
                path_effects=[path_effects.SimpleLineShadow(),path_effects.Normal()]) # Add a shadow
                )

    # Show the trolley image for the unshaped case
    ax1.imshow(
            TROLLEY_IMAGE, # Use the previously loaded image
            interpolation='bilinear',
            extent=[ # Define the extents by the four corners of the image
                unshaped_trolley_x[i] - TROLLEY_OFFSET_X, # Minimum X value
                unshaped_trolley_x[i] + TROLLEY_OFFSET_X, # Maximum X value
                -TROLLEY_OFFSET_Y, # Minimum Y value
                TROLLEY_OFFSET_Y], # Maximum Y value
            aspect='equal') # Do not stretch the image. This overrides the axis limits

    # Show the trolley image for the shaped case
    ax1.imshow(
            TROLLEY_IMAGE,
            interpolation='bilinear', # Use the previously loaded image
            extent=[ # Define the extents by the four corners of the image
                shaped_trolley_x[i] - TROLLEY_OFFSET_X, # Minimum X value
                shaped_trolley_x[i] + TROLLEY_OFFSET_X, # Maximum X value
                -TROLLEY_OFFSET_Y, # Minimum Y value
                TROLLEY_OFFSET_Y], # Maximum Y value
            aspect='equal') # Do not stretch the image. This overrides the axis limits

    # Set the alpha as 1 or 0 based on whether "OPAQUE" is True or False
    fig.patch.set_alpha(float(OPAQUE))

# Create the animation
anim = FuncAnimation(
                fig, # Use the predefined figure
                update, # Call the update function
                frames=fps * int(np.amax(time)), # Use a number of frames based on the framerate and length of the time array
                interval=fps)

# anim.save(
#     'crane_anim.mov', # Set the file name
#     codec="png",
#     dpi=100, 
#     bitrate=-1, 
#     savefig_kwargs={
#                 'transparent': TRANSPARENT, 
#                 'facecolor': 'none'})

# Added: 03/31/18 - Joshua Vaughan - joshua.vaughan@louisiana.edu
# 
# I had a hard time getting good quality videos using Daniel's settings, without
# reprocessing the video in QuickTime. I'm guessing that we have different sets 
# of video codecs installed. Below is what gave me the best results on my home 
# iMac.
# 
# These encoder I'm using (h264) also doesn't seem to get along with the video
# being transparent. So, this version will have a white background. This also 
# may make it better as a "standalone" video. For me, these settings also seem
# to result in smaller filesizes for approximately the same quality. 
FFMpegWriter = writers['ffmpeg']

# We can also add some metadata to the video.
metadata = dict(title='Input Shaping Animation', artist='CRAWLAB',
                comment='Shows a point-to-poitn move of a planar crane with and without input shaping.')

# Change the video bitrate as you like and add some metadata.
writer = FFMpegWriter(codec="h264", fps=fps, bitrate=-1, metadata=metadata)

anim.save(
    'crane_anim.mp4', # Set the file name
    dpi=240,          # Bump up to 4K resolution 3840x2160
    writer=writer,
    savefig_kwargs={
                'transparent': False, # h264 doesn't seem to like transparency 
                'facecolor': 'none'})


# plt.show() # JEV - This seemed to force encoding into an endless loop for me
plt.close()