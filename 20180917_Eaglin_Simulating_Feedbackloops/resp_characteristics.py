#!/usr/bin/env python

# resp_characteristics.py
#
# Module containing functions for determining characteristics of a response
# 		Originally written for characterizing the response of an electromagnetic actuator
#
#
# Written by Gerald Eaglin, ULL
# February 2018

import numpy as np

def rise(resp,t,initial,setpoint,percent):
    """Determine the rise time of a response"""
    for i in range(len(t)):
        # if resp[i] > percent*(final-initial):
        if (resp[i]-initial)/(setpoint-initial) > percent:
            rise = t[i-1] - t[0]
            break

    return rise



def settling(resp,t,initial,setpoint,percent):
    """Determine the settling time of a response"""
    settling=None
    for i in range(2,len(t)):
        if np.absolute(resp[-i]-setpoint)/(setpoint-initial) > percent:
            settling=t[-i+1] - t[0]
            break

    return settling




def overshoot(resp,t,initial,setpoint):
    """Given the response, calculate the percent overshoot"""

    ov=max(resp) - initial # overshoot
    step=setpoint-initial

    if ov-step>=0:
        # overshoot occurs
        PO=(ov-step)/step # percent overshoot
    else:
        # no overshoot occurs
        PO=0

    return PO




