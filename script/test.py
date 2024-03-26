#!/usr/bin/env python3

import rospy
import time
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

from tvmc import MotionController, DoF, ControlMode

HEAVE_KP = 153
HEAVE_KI = 0
HEAVE_KD = 51
HEAVE_TARGET = 2.5
HEAVE_ACCEPTABLE_ERROR = 0.05

PITCH_KP = -1.5
PITCH_KI = -0
PITCH_KD = -0.8
PITCH_TARGET = 0
PITCH_ACCEPTABLE_ERROR = 1.5

YAW_KP = -0.6
YAW_KI = 0
YAW_KD = -0.25
YAW_ACCEPTABLE_ERROR = 2

g_relPos = Vector3()  
yaw_curr_point=0

def update_gate(x):
    global g_relPos
    g_relPos = x

def orientation(x):
    global yaw_curr_point
    yaw_curr_point=x.x
    m.set_current_point(DoF.YAW, x.x)
    m.set_current_point(DoF.PITCH, x.y)

def depth(d):
    m.set_current_point(DoF.HEAVE, d.data)

#start motion controller
m = MotionController()
m.start()

#yaw pid
m.set_control_mode(DoF.YAW, ControlMode.CLOSED_LOOP)
m.set_pid_constants(DoF.YAW, YAW_KP, YAW_KI, YAW_KD, YAW_ACCEPTABLE_ERROR)
m.set_pid_limits(DoF.YAW, -10, 10, -25, 25)

#pitch pid
m.set_control_mode(DoF.PITCH, ControlMode.CLOSED_LOOP)
m.set_pid_constants(DoF.PITCH, PITCH_KP, PITCH_KI, PITCH_KD, PITCH_ACCEPTABLE_ERROR)
m.set_pid_limits(DoF.PITCH, -10, 10, -25, 25)
m.set_target_point(DoF.PITCH, PITCH_TARGET)

#heave pid
m.set_control_mode(DoF.HEAVE, ControlMode.CLOSED_LOOP)
m.set_pid_constants(DoF.HEAVE, HEAVE_KP, HEAVE_KI, HEAVE_KD, HEAVE_ACCEPTABLE_ERROR)
m.set_pid_limits(DoF.HEAVE, -10, 10, -25, 25)
m.set_target_point(DoF.HEAVE, HEAVE_TARGET)

#surge openloop
m.set_control_mode(DoF.SURGE, ControlMode.OPEN_LOOP)

#subscribers
rospy.Subscriber("/emulation/orientation", Vector3, orientation)
rospy.Subscriber("/emulation/depth", Float64, depth)
rospy.Subscriber("/localization/gate", Vector3, update_gate)

rate = rospy.Rate(5)

while rospy.is_shutdown() == False:
    
    if(g_relPos.x!=0):
        print("Angle :",(math.atan(g_relPos.y/g_relPos.x))*(180/math.pi))
        m.set_target_point(DoF.YAW, (math.atan(g_relPos.y/g_relPos.x))*(180/math.pi))
        m.set_thrust(DoF.SURGE, -70)    
    else:
        print("Searching...")
        m.set_target_point(DoF.YAW, yaw_curr_point+3)
    '''
    if(g_relPos.x!=0 and g_relPos.y!=0):
        print(g_relPos.x,'\t',g_relPos.y,'\t',g_relPos.z)
        if(g_relPos.x<-0.5):
            print("enter right")
            m.set_thrust(DoF.SURGE, 0) 
            m.set_current_point(DoF.YAW, +5)
        elif(g_relPos.x>0.5):
            print("enter left")
            m.set_thrust(DoF.SURGE, 0) 
            m.set_current_point(DoF.YAW, -5)
        elif(g_relPos.x>-0.5 and g_relPos.x<0.5):
            m.set_current_point(DoF.YAW, 0)
            m.set_thrust(DoF.SURGE, -70)    
    else :
        m.set_thrust(DoF.SURGE, 0)
        m.set_current_point(DoF.YAW, 10)
    '''
    rate.sleep()
