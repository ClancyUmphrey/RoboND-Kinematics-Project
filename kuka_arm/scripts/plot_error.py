#!/usr/bin/env python

# File: plot_error.py

# Description:
# Subscribes to the ee_error topic and plots the data array each callback.

# Author: Clancy Umphrey

from matplotlib import pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import warnings
warnings.filterwarnings("ignore")

import rospy
from kuka_arm.msg import Float64List


def init_plot():
    """
    Initializes and refreshes the plot format and characteristics.
    """
    plt.title('End Effector Position Error Magnitude')
    plt.xlabel('Path Pose Index')
    plt.ylabel('Error (1E-9)')
    ax = plt.gca()
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
    ax.xaxis.set_major_formatter(FormatStrFormatter('%d'))
    

def callback(msg):
    """
    Clear the axes and plot the new error data.
    """
    plt.cla() # clear the current axes
    init_plot()
    plt.plot([d * 1e9 for d in msg.data])
    plt.draw()
    plt.pause(1e-11)


def close_plot():
    """
    Shutdown-hook that closes the plot.
    """
    print "\nClosing plot."
    plt.close()


if __name__ == '__main__':
    # ROS Initialization
    rospy.init_node("plot_error")
    rospy.on_shutdown(close_plot)
    rospy.Subscriber("ee_error", Float64List, callback)

    # Plotting Initialization
    print 'Ready to plot end effector error.'
    # Display plot without data and with arbitrary axis values.
    init_plot()
    # Block to prevent script from exiting.
    plt.show(block=True)
    # NOTE: Using "plot.show(block=True)" instead of "rospy.spin()" to prevent
    # the script from exiting avoids the following error on shutdown:
    # "RuntimeError: main thread is not in main loop"
    # that arises from having two threads running--rospy.spin() and the
    # matplotlib GUI. See the first reference below.


# References:
# https://stackoverflow.com/questions/35145555/python-real-time-plotting-ros-data
# https://answers.ros.org/question/264767/plotting-real-time-data/
