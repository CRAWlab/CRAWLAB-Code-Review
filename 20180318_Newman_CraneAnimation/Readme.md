# Creating Presentation-Ready Animations using Matplotlib

This script describes the process by which you can create clean animations using Python.

Let's start by importing the necessary libraries.
```python
import numpy as np
import matplotlib.pylab as plt
import scipy
from scipy import ndimage
import matplotlib.patheffects as path_effects
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D
from matplotlib.animation import FuncAnimation
```

Next, let's define some constants that we will use throughout this script.

These color strings will be used to create the desired colors for the shaped and unshaped plots
```python
# Maintain consistency between different plot colors
UNSHAPED_COLOR = '#e41a1c'
SHAPED_COLOR = '#377eb8'
```

To put our animation on a powerpoint or keynote presentation, it will be helpful to have the option of making them transparent.
```python
# Determine whether we want to make a transparent background 
# for clearner presentation
OPAQUE = False
TRANSPARENT = not OPAQUE
```

When plotting on matplotlib, I like to use multipliers to determine my minimum and maximum windows:
```python
# These are some constants for my plotting
# I have multiplicative factors to give decent room above 
# and below the plotted data, making my figures look cleaner
Y_MIN = 0.1
Y_MAX = 0.3
```

We will be using a loaded image "Trolley.png". This file is 200x200 and therefore its center is located by:
```python
# Locate the center of the trolley image at these coordinates
TROLLEY_OFFSET_X = 100
TROLLEY_OFFSET_Y = 100
```

This larger image will require us to scale-up the responses
```python
# Because we are importing an external image, we need to scale the
# response values to match the size of the image
SCALING_FACTOR = 500
```

Now let's define the trolley image:
```python
# Load an external image for the trolley
TROLLEY_IMAGE = plt.imread("Trolley.png")
```

We will define two plots which will have axis labels defined by:
```python
# Specify axis labels
X_LABEL = 'Time (s)'
Y1_LABEL = 'Trolley Velocity (m/s)'
Y2_LABEL = 'Payload Angle (deg)'
```

## Load the relevant response values

First, let's load the time array.
```python
# Let's load response data from a previously run simulation
time = np.load('time.npy')
```

Because we're creating an animation from this response, we can use the time array to determine the framerate of the animation.
```python
# Framerate of the animation, derived from the sampling time 
# of the simulation
fps = int(np.round(1 / (time[1] - time[0])))
```

Now let's load and extract the relevant response data for the unshaped and shaped commands:
```python

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
```

## Create the figure area

Let's create a large figure and create three axes where we can display the different plots we want to show.
```python
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
```

## Initialize null plot lines

Let's create lines where we will plot the response data.
```python
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
```

## Format the plot axes

On ax2 and ax3, we will be plotting the response data. Let's format those axes according to standard CRAWLAB formatting.

```python
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
```

Convert the responses to their respective cable x and y coordinates

```python
# Set the X and Y endpoints for the unshaped cable
unshaped_cable_end_x = unshaped_trolley_x - length * np.sin(unshaped_theta)
unshaped_cable_end_y = - length * np.cos(unshaped_theta)

# Set the X and Y endpoints for the shaped cable
shaped_cable_end_x = shaped_trolley_x - length * np.sin(shaped_theta)
shaped_cable_end_y = - length * np.cos(shaped_theta)
```

Now let's create an animation function. This function will be called for every frame.
```python
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
```

Let's create and save the animation by calling the update function:
```python
# Create the animation
anim = FuncAnimation(
                fig, # Use the predefined figure
                update, # Call the update function
                frames=fps * int(np.amax(time)), # Use a number of frames based on the framerate and length of the time array
                interval=fps)

anim.save(
    'crane_anim.mov', # Set the file name
    codec="png",
    dpi=100, 
    bitrate=-1, 
    savefig_kwargs={
                'transparent': TRANSPARENT, 
'facecolor': 'none'})
```

Show the result before closing.
```python
plt.show()
plt.close()
```