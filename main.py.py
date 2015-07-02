# -*- encoding: UTF-8 -*-
###############################################################################
### This programe is used to send joints evolution to NAO
###############################################################################
import sys
import time
from naoqi import ALProxy
import almath
import csv

###############################################################################
### Main function
###############################################################################
def main(robotIP):

    PORT = 9559
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    
    # Set stiffness and let robot go to initial posture    
    motionProxy.setStiffnesses("Body", 1.0)
    postureProxy.goToPosture("StandInit", 1.0)
    
    
################################################################################
### q_total.csv -------> NAO
################################################################################
    
    # Example showing how to get the names of all the joints in the body.
    BodyNames =  ["RHipYawPitch",
              "RHipRoll",
              "RHipPitch",
              "RKneePitch",
              "RAnklePitch",
              "RAnkleRoll",

              "LHipYawPitch",
              "LHipRoll",
              "LHipPitch",
              "LKneePitch",
              "LAnklePitch",
              "LAnkleRoll",

              "RShoulderPitch",
              "RShoulderRoll",
              "RElbowYaw",
              "RElbowRoll",
              "RWristYaw",

              "LShoulderPitch",
              "LShoulderRoll",
              "LElbowYaw",
              "LElbowRoll",
              "LWristYaw",

              "HeadYaw",
              "HeadPitch"]
     
  # Read jointProfiles data from .csv file and store in a cooresponding lists
    angleLists=[]
    timeLists=[]
    with open('trajectory.csv', 'rb') as csvfile: #with open('diagonal.csv', 'rb') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        index=1
        for row in spamreader:
            angleLists.append([float(x) for x in row])

    timelist = []
    #####################################
    # Bevare to put correct sampling time!!!!!!
    #####################################
    sample_time =0.05 #same value as time in generate_step.m
    for i in range(len(angleLists[0])):
        timelist.append(i*sample_time+sample_time)

    # duplicate timeLists for each joint
    for i in range(24):
        timeLists.append(timelist)

    # Extract the initial state of the robot form angleLists
    MyStandInit=[]
    for row in angleLists:
        MyStandInit.append(row[0])
    motionProxy.setAngles(BodyNames, MyStandInit, 0.5)
    time.sleep(2)
  
    isAbsolute = True
    motionProxy.angleInterpolation(BodyNames, angleLists, timeLists, isAbsolute)

	 
    postureProxy.goToPosture("Sit", 1.0)
    motionProxy.setStiffnesses("Body", 0.0)
    
###################################
###          Main program        ##
###################################
if __name__ == "__main__":
   # For the real robot
   robotIp = "192.168.0.11"
   
   # For the simulated robot
   #robotIp = "127.0.0.1"
   
   main(robotIp)
