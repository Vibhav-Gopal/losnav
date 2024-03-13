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


def depth(d):
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


#POV @ Marty!!
while rospy.is_shutdown() == False:
    print(f_relPos.x,f_relPos.y)
    if(f_relPos.y<1.5):  #if flare is within 1.5 depth in front of me i will avoid it 
        if(f_relPos.x>-0.5 and f_relPos.x<0):   #flare towards my left
            if(f_relPos.x!=0 and f_relPos.y!=0): #since zero is default value while not finding flare at start we want to avoid that case
                m.set_current_point(DoF.YAW,0) #point to note: dont put this in parent if condition otherwise it will loop at getting beginning 0 for flare(not detected) and bot will remain stationary
                #wait why the above line?....because if i dont detect anything and keep yawing and suddenly see nearby flare
                if(g_relPos.x>-0.4 and g_relPos.x<0):   #if both flare and gate are towards my left
                    print("detect left flare and gate")
                    print("flare :",f_relPos.x)
                    print("gate :",g_relPos.x)
                    m.set_thrust(DoF.SURGE, +70)
                    time.sleep(1)
                    m.set_thrust(DoF.SURGE, 0)
                    m.set_current_point(DoF.YAW, -5)
                    print("yawwweee")
                    time.sleep(1)
                    m.set_current_point(DoF.YAW, 0)
                    print("surgeeeeee")
                    m.set_thrust(DoF.SURGE, -70)
                    time.sleep(5)
                    m.set_thrust(DoF.SURGE, 0)
                    continue  #point to note: do not put this continue in the parent if condition because then at getting beginning 0 of flare(not detected) it will just keep looping and bot will remain stationary 
                elif(g_relPos.x<0.4 and g_relPos.x>0): #if flare is towards my left but gate towards my right
                    print("detect left flare right gate")
                    print("flare :",f_relPos.x)
                    print("gate :",g_relPos.x)
                    m.set_thrust(DoF.SURGE, +70)
                    time.sleep(1)
                    m.set_thrust(DoF.SURGE, 0)
                    m.set_current_point(DoF.YAW, -5)
                    print("yawwweee")
                    time.sleep(1)
                    m.set_current_point(DoF.YAW, 0)
                    print("surgeeeeee")
                    m.set_thrust(DoF.SURGE, -70)
                    time.sleep(5)
                    m.set_thrust(DoF.SURGE, 0)
                    continue
                else:                           #if i see only flare nearby and gate is far away in regards of horizontal direction
                    print("detect left flare")
                    print("flare :",f_relPos.x)
                    #print("gate :",g_relPos.x)
                    m.set_thrust(DoF.SURGE, +70)
                    time.sleep(2)
                    m.set_thrust(DoF.SURGE, 0)
                    m.set_current_point(DoF.YAW, +10)
                    time.sleep(1)
                    m.set_current_point(DoF.YAW, 0)
                    m.set_thrust(DoF.SURGE, -70)
                    time.sleep(2)
                    m.set_thrust(DoF.SURGE, 0)
                    continue
        elif(f_relPos.x>0 and f_relPos.x<0.5):  #flare towards my right
            m.set_current_point(DoF.YAW,0)
            if(g_relPos.x>-0.4 and g_relPos.x<0): #if flare is towards my right but gate is towards my left 
                print("detect right flare and left gate")
                print("flare :",f_relPos.x)
                print("gate :",g_relPos.x)
                m.set_thrust(DoF.SURGE, +70)
                time.sleep(1)
                m.set_thrust(DoF.SURGE, 0)
                m.set_current_point(DoF.YAW, +5)
                print("yawwweee")
                time.sleep(1)
                m.set_current_point(DoF.YAW, 0)
                print("surgeeeeee")
                m.set_thrust(DoF.SURGE, -70)
                time.sleep(5)
                m.set_thrust(DoF.SURGE, 0)
                continue
            elif(g_relPos.x<0.4 and g_relPos.x>0): #if both flare and gate towards my right
                print("detect right flare and gate")
                print("flare :",f_relPos.x)
                print("gate :",g_relPos.x)
                m.set_thrust(DoF.SURGE, +70)
                time.sleep(1)
                m.set_thrust(DoF.SURGE, 0)
                m.set_current_point(DoF.YAW, +5)
                print("yawwweee")
                time.sleep(1)
                m.set_current_point(DoF.YAW, 0)
                print("surgeeeeee")
                m.set_thrust(DoF.SURGE, -70)
                time.sleep(5)
                m.set_thrust(DoF.SURGE, 0)
                continue
            else:                           #if i see only flare nearby and gate is far away in regards of horizontal direction
                print("detect right flare")
                print("flare :",f_relPos.x)
                #print("gate :",g_relPos.x)
                m.set_thrust(DoF.SURGE, +70)
                time.sleep(2)
                m.set_thrust(DoF.SURGE, 0)
                m.set_current_point(DoF.YAW, -10)
                time.sleep(1)
                m.set_current_point(DoF.YAW, 0)
                m.set_thrust(DoF.SURGE, -70)
                time.sleep(2)
                m.set_thrust(DoF.SURGE, 0)
                continue
        
    if(g_relPos.x!=0 and g_relPos.y!=0):    #When my flare is not 1.5 in front of me i will go ahead towards gate even if flare is in front of me
        #print(g_relPos.x,'\t',g_relPos.y,'\t',g_relPos.z)
        if(g_relPos.x<-0.4):                #i will yaw towards left if gate is too much in the left
            print("no detect right gate")
            m.set_thrust(DoF.SURGE, 0) 
            m.set_current_point(DoF.YAW, +3)
        elif(g_relPos.x>0.4):               #i will yaw towards right if gate is too much in the right
            print("no detect left gate")
            m.set_thrust(DoF.SURGE, 0) 
            m.set_current_point(DoF.YAW, -3)
        elif(g_relPos.x>-0.4 and g_relPos.x<0.4):   #gate spotted in center, HIT THE ACCELERATOR!!!
            print("detect gate center")
            m.set_current_point(DoF.YAW, 0)
            m.set_thrust(DoF.SURGE, -70)    
    else :
        print("no no detect gate")      #oh shit! i don't see gate anywhere, i need to look around!
        m.set_thrust(DoF.SURGE, 0)
        m.set_current_point(DoF.YAW, 10)    #only yaw implemented, think about including both yaw and surge
    rate.sleep()


    #Shout out to Vibhav.....
    #Need a start point to finish it
    #Shout out to Rose and Satvik....
    #I dont think i would have been able to come up with this scripit without testing it out on the simulator