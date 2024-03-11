#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3


from tvmc import MotionController, DoF, ControlMode


g_relPos = Vector3()
f_relPos = Vector3()    

def update_gate(x):
    
    global g_relPos
    g_relPos = x
    #print(relPos.x, "\t", relPos.y, "\t", relPos.z)

def update_flare(x):
    global f_relPos
    f_relPos = x
    #print(relPos.x, "\t", relPos.y, "\t", relPos.z)

YAW_KP = -0.6
YAW_KI = -0.04
YAW_KD = -0.005
YAW_TARGET = 0
YAW_ACCEPTABLE_ERROR = 0.05

HEAVE_KP = 100
HEAVE_KI = 13
HEAVE_KD = 0
HEAVE_TARGET = 2.5
HEAVE_ACCEPTABLE_ERROR = 0.05

curr_yaw = 0


def orientation(x):
    global curr_yaw
    #m.set_current_point(DoF.ROLL, x.x)
    #m.set_current_point(DoF.PITCH, x.y)
    #m.set_current_point(DoF.YAW, x.z + 180)
    #curr_yaw = x.z + 180
    #print("Curr Yaw :",curr_yaw,end='\r')


def depth(d):
    # print("Depth : ", d.data, end="\r")
    m.set_current_point(DoF.HEAVE, d.data)


m = MotionController()
m.start()
m.set_control_mode(DoF.YAW, ControlMode.CLOSED_LOOP)
m.set_control_mode(DoF.HEAVE, ControlMode.CLOSED_LOOP)
m.set_pid_constants(DoF.HEAVE, HEAVE_KP, HEAVE_KI, HEAVE_KD, HEAVE_ACCEPTABLE_ERROR)
m.set_pid_constants(DoF.YAW, YAW_KP, YAW_KI, YAW_KD, YAW_ACCEPTABLE_ERROR)
m.set_pid_limits(DoF.HEAVE, -2, 2, -50, 50)
m.set_target_point(DoF.HEAVE, HEAVE_TARGET)
m.set_current_point(DoF.HEAVE, 0.09)

rospy.Subscriber("/diagnostics/orientation", Vector3, orientation)
rospy.Subscriber("/depth_data", Float64, depth)
rospy.Subscriber("/localization/gate", Vector3, update_gate)
rospy.Subscriber("/localization/flare", Vector3, update_flare)

m.set_control_mode(DoF.YAW, ControlMode.CLOSED_LOOP)
m.set_control_mode(DoF.SURGE, ControlMode.OPEN_LOOP)
rate = rospy.Rate(5)
import math

time.sleep(1)
#rospy.spin()

while rospy.is_shutdown() == False:

    if(f_relPos.x>-0.5 and f_relPos.y<0.5):
        if(f_relPos.x!=0 and f_relPos.y!=0):
            m.set_thrust(DoF.SURGE, +70)
            time.sleep(1)
            m.set_thrust(DoF.SURGE, 0)
            m.set_current_point(DoF.YAW, 10)
    if(g_relPos.x!=0 and g_relPos.y!=0):
        print(g_relPos.x,'\t',g_relPos.y,'\t',g_relPos.z)
        if(g_relPos.x<-0.8):
            print("enter right")
            m.set_thrust(DoF.SURGE, 0) 
            m.set_current_point(DoF.YAW, +5)
        elif(g_relPos.x>0.8):
            print("enter left")
            m.set_thrust(DoF.SURGE, 0) 
            m.set_current_point(DoF.YAW, -5)
        elif(g_relPos.x>-0.8 and g_relPos.x<0.8):
            m.set_current_point(DoF.YAW, 0)
            m.set_thrust(DoF.SURGE, -70)    
    else :
        m.set_thrust(DoF.SURGE, 0)
        m.set_current_point(DoF.YAW, 10)
    rate.sleep()
