#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Float64
from geometry_msgs.msg import Vector3
from pathlib import Path
import cv2

from tvmc import MotionController, DoF, ControlMode

# nnBlobPath = str((Path(__file__).parent / Path('../models/my_openvino_blob.blob')).resolve().absolute())
relPos = Vector3()


def update_curr_state(x):
    #print(relPos.x, "\t", relPos.y, "\t", relPos.z)
    global relPos
    relPos = x


# if not Path(nnBlobPath).exists():
#     import sys
#     raise FileNotFoundError(f'Required file/s not found')

# labelMap = [
#     "Cans",
#     "bottle",
#     "paper"
# ]

# syncNN = True

YAW_KP = -0.6
YAW_KI = -0.04
YAW_KD = 0.0
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
    m.set_current_point(DoF.ROLL, x.x)
    m.set_current_point(DoF.PITCH, x.y)
    m.set_current_point(DoF.YAW, x.z + 180)
    curr_yaw = x.z + 180
    # print("Curr Yaw :",curr_yaw,end='\r')


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
#m.set_target_point(DoF.YAW, YAW_TARGET)

rospy.Subscriber("/diagnostics/orientation", Vector3, orientation)
rospy.Subscriber("/depth_data", Float64, depth)
rospy.Subscriber("/localization/gate", Vector3, update_curr_state)

m.set_control_mode(DoF.YAW, ControlMode.CLOSED_LOOP)
m.set_control_mode(DoF.SURGE, ControlMode.OPEN_LOOP)
m.set_thrust(DoF.SURGE, -70)
# m.set_thrust(DoF.YAW, 2)#
rate = rospy.Rate(5)
import math

curr_target = math.degrees(math.atan2(relPos.y, relPos.x)) -90
m.set_target_point(DoF.YAW, YAW_TARGET)
time.sleep(1)
#curr_target = curr_yaw

# curr_target = curr_target + 90
# m.set_target_point(DoF.YAW,320)
# rospy.spin()

while rospy.is_shutdown() == False:
    """if math.degrees(math.atan2(relPos.y, relPos.x)) == 0:
    curr_target =  math.degrees(math.atan2(relPos.y, relPos.x))
    m.set_target_point(DoF.YAW, curr_target)
    #print(curr_target)"""
    print(relPos.x, "\t", relPos.y, "\t", relPos.z)
    #if relPos.z > 0:
    #    print("yes")
    #if relPos.z < 0:
    #    print("no")
    # if math.degrees(math.atan2(relPos.y, relPos.x)) < 150:
    # curr_target = min(curr_target + .5, +180)
    #curr_target = math.degrees(math.atan2(relPos.y, relPos.x)) -90 for 1
    #curr_target = math.degrees(math.atan2(relPos.y, relPos.x)) - 80 for 2
    #curr_target = math.degrees(math.atan2(relPos.y, relPos.x)) - 90 for 3
    #curr_target = math.degrees(math.atan2(relPos.y, relPos.x)) - 90 for 4
    if(relPos.x!=0 and relPos.y!=0):
        m.set_thrust(DoF.SURGE, -70)
        curr_target = math.degrees(math.atan2(relPos.y, relPos.x)) -90
        m.set_target_point(DoF.YAW, curr_target)
    else :
        m.set_thrust(DoF.SURGE, 0)
        YAW_TARGET = YAW_TARGET + 20
        m.set_target_point(DoF.YAW, YAW_TARGET) 

    #print(relPos.x, "\t", relPos.y, "\t", relPos.z)
    # elif math.degrees(math.atan2(relPos.y, relPos.x)) > 150:
    #   m.set_target_point(DoF.YAW, curr_target)
    # curr_target = math.degrees(math.atan2(relPos.y, relPos.x)) +100
    # curr_target = max(curr_target - .5, -180)'''
    # if curr_target<60 : curr_target+=.1
    # m.set_target_point(DoF.YAW, curr_target)
    rate.sleep()
