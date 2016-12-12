#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */



import hubo_ach as ha
import ach
import sys
import time
from ctypes import *
import math
import numpy as np

LEFT_ARM = 0
RIGHT_ARM = 1

#Length of arm components
LENGTH_BASE_TO_SHOULDER  = 214.5 - 120.0
LENGTH_SHOULDER_TO_ELBOW = 179.14
LENGTH_ELBOW_TO_WRIST    = 181.59
LENGTH_WRIST_TO_FINGER   = 0.0 #placeholder
#Length of arm components
LENGTH_BASE_TO_SHOULDER  = 214.5 - 120.0
LENGTH_SHOULDER_TO_ELBOW = 179.14
LENGTH_ELBOW_TO_WRIST    = 181.59
LENGTH_WRIST_TO_FINGER   = 0.0 #placeholder

#IK Constatns
DELTA_THETA = 0.01
ERROR       = 5 # PLACEHOLDER - ERROR FROM GOAL
ENDEFF_STEP = 15 # PLACEHOLDER - STEP_SIZE

#GOALS LEFT ARM
GOAL_X_OFFSET = 10.09298
LEFT_GOAL_TOP_LEFT  = np.array([[LENGTH_SHOULDER_TO_ELBOW + LENGTH_ELBOW_TO_WRIST - GOAL_X_OFFSET], [34.5], [60.0]])
LEFT_GOAL_BOT_LEFT  = np.array([[LENGTH_SHOULDER_TO_ELBOW + LENGTH_ELBOW_TO_WRIST - GOAL_X_OFFSET], [34.5], [-60.0]])
LEFT_GOAL_TOP_RIGHT = np.array([[LENGTH_SHOULDER_TO_ELBOW + LENGTH_ELBOW_TO_WRIST - GOAL_X_OFFSET], [154.5], [60.0]])
LEFT_GOAL_BOT_RIGHT = np.array([[LENGTH_SHOULDER_TO_ELBOW + LENGTH_ELBOW_TO_WRIST - GOAL_X_OFFSET], [154.5], [-60.0]])

#GOALS RIGHT ARM

RIGHT_GOAL_TOP_LEFT  = np.array([[LENGTH_SHOULDER_TO_ELBOW + LENGTH_ELBOW_TO_WRIST - GOAL_X_OFFSET], [-154.5], [60.0]])
RIGHT_GOAL_BOT_LEFT  = np.array([[LENGTH_SHOULDER_TO_ELBOW + LENGTH_ELBOW_TO_WRIST - GOAL_X_OFFSET], [-154.5], [-60.0]])
RIGHT_GOAL_TOP_RIGHT = np.array([[LENGTH_SHOULDER_TO_ELBOW + LENGTH_ELBOW_TO_WRIST - GOAL_X_OFFSET], [-34.5], [60.0]])
RIGHT_GOAL_BOT_RIGHT = np.array([[LENGTH_SHOULDER_TO_ELBOW + LENGTH_ELBOW_TO_WRIST - GOAL_X_OFFSET], [-34.5], [-60.0]])
RIGHT_GOAL_BOT_RIGHT = np.array([[LENGTH_SHOULDER_TO_ELBOW + LENGTH_ELBOW_TO_WRIST - GOAL_X_OFFSET], [-34.5], [-60.0]])

#CENTER

RIGHT_GOAL_TOP_STRAIGHT  = np.array([[LENGTH_SHOULDER_TO_ELBOW + LENGTH_ELBOW_TO_WRIST- 76.6095], [120.0], [60.0]])

LEFT_LEG = 0
RIGHT_LEG = 1

LOWER_LEG = 300.38
UPPER_LEG = 300.03
ANKLE_LENGTH = 94.97

#Squat
SQUAT_LENGTH = 75.0
SQUAT_DOWN_Y = -1.0 * (LOWER_LEG + UPPER_LEG - SQUAT_LENGTH)
SQUAT_DOWN_X = 0.0

#Forward
LIFT_UP = 125.0
STEP_SIZE = -50.0
WALK_FORWARD_LEG_Y = -1.0 * (LOWER_LEG + UPPER_LEG - LIFT_UP)
WALK_FORWARD_LEG_X = STEP_SIZE

#LEG DOWN
WALK_DOWN_LEG_Y = SQUAT_DOWN_Y
WALK_DOWN_LEG_X = STEP_SIZE

LEAN_ANGLE = 0.125 

########################################################################

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
#s.flush()
#r.flush()

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=True)

#initial Theta values
leftThetaInit = np.zeros((6,1))
newLeftThetas = np.zeros((6,1))
rightThetaInit = np.zeros((6,1))

########################################################################

def legsInverseKinematics(x,y,d1,d2):
	thetaKnee = math.acos((x*x+y*y-d1*d1-d2*d2)/(2.0*d1*d2))
	thetaHip = math.atan2(y*(d1+d2*math.cos(thetaKnee)) - x*d2*math.sin(thetaKnee) ,  x*(d1+d2*math.cos(thetaKnee)) + y*d2*math.sin(thetaKnee) )

	thetaHip = -1.0*(math.pi/2.0 - thetaHip)
	
	
	if(thetaHip > math.pi):
		thetaHip = thetaHip - math.pi
	if(thetaHip < -1.0 * math.pi):
		thetaHip = thetaHip + math.pi
	if(thetaKnee > math.pi):
		thetaKnee = thetaKnee - math.pi
	if(thetaKnee < -1.0 * math.pi):
		thetaKnee = thetaKnee + math.pi

	thetaAnkle = -1.0 * (180.0 + thetaHip - (180.0 - thetaKnee))
	if(thetaAnkle > math.pi):
		thetaAnkle = thetaAnkle - math.pi
	if(thetaAnkle < -1.0 * math.pi):
		thetaAnkle = thetaAnkle + math.pi
	
	print "thetaHip   (RHP,LHP) = ", math.degrees(thetaHip)
	print "thetaKnee  (RKN,LKN) = ", math.degrees(thetaKnee)
	print "thetaAnkle (RAP,LAP) = ", math.degrees(thetaAnkle)

	return (thetaHip, thetaKnee, thetaAnkle)

def squat():
	
	thetaHip, thetaKnee, thetaAnkle = legsInverseKinematics(SQUAT_DOWN_X, SQUAT_DOWN_Y, UPPER_LEG, LOWER_LEG)
	
	ref.ref[ha.RKN] = thetaKnee
	ref.ref[ha.LKN] = thetaKnee

	ref.ref[ha.RHP] = thetaHip
	ref.ref[ha.LHP] = thetaHip

	ref.ref[ha.RAP] = thetaAnkle
	ref.ref[ha.LAP] = thetaAnkle 

	# Write to the feed-forward channel
	r.put(ref)
	simSleep(1.0)
	
	return
	
def walkLeanRight():
	
	step = 0.0
	step_size = 0.01

	while(step < (LEAN_ANGLE - step_size)):
		ref.ref[ha.RAR] = -step 
		ref.ref[ha.LAR] = -step 

		ref.ref[ha.RHR] = step 
		ref.ref[ha.LHR] = step 
	
		r.put(ref)
		
		step = step + step_size
		
		simSleep(.1) #time.sleep(0.1)

		print "Lean Right : Lean Angle = ", math.degrees(LEAN_ANGLE), "Current Angle = ", math.degrees(step)
	
	ref.ref[ha.RAR] = -LEAN_ANGLE
	ref.ref[ha.LAR] = -LEAN_ANGLE

	ref.ref[ha.RHR] = LEAN_ANGLE
	ref.ref[ha.LHR] = LEAN_ANGLE	

	r.put(ref)

	simSleep(.2) 
	
	return
	
def walkLeanLeft():
	
	step = 0.0
	step_size = 0.01

	while(step < (LEAN_ANGLE - step_size)):
		ref.ref[ha.RAR] = step 
		ref.ref[ha.LAR] = step 

		ref.ref[ha.RHR] = -step 
		ref.ref[ha.LHR] = -step 
	
		r.put(ref)
		
		step = step + step_size
		
		simSleep(.05) 

		print "Lean Left : Lean Angle = ", math.degrees(LEAN_ANGLE), "Current Angle = ", math.degrees(step)
	
	ref.ref[ha.RAR] = LEAN_ANGLE
	ref.ref[ha.LAR] = LEAN_ANGLE

	ref.ref[ha.RHR] = -LEAN_ANGLE
	ref.ref[ha.LHR] = -LEAN_ANGLE	
	
	r.put(ref)

	simSleep(.2) 
	
	return
	
	
def stepLeg(leg):
	
	####################################################################
	print "moving leg forward"
	thetaHip, thetaKnee, thetaAnkle = legsInverseKinematics(WALK_FORWARD_LEG_X, WALK_FORWARD_LEG_Y, UPPER_LEG, LOWER_LEG)
	if (leg == LEFT_LEG):
		ref.ref[ha.LKN] = thetaKnee
		ref.ref[ha.LHP] = thetaHip
		ref.ref[ha.LAP] = thetaAnkle
	if (leg == RIGHT_LEG):
		ref.ref[ha.RKN] = thetaKnee
		ref.ref[ha.RHP] = thetaHip
		ref.ref[ha.RAP] = thetaAnkle 
		
	r.put(ref)
	simSleep(0.4)
	
	####################################################################
	print "moving leg down"
	thetaHip, thetaKnee, thetaAnkle = legsInverseKinematics(WALK_DOWN_LEG_X, WALK_DOWN_LEG_Y, UPPER_LEG, LOWER_LEG)
	if (leg == LEFT_LEG):
		ref.ref[ha.LKN] = thetaKnee
		ref.ref[ha.LHP] = thetaHip
		ref.ref[ha.LAP] = thetaAnkle 
	if (leg == RIGHT_LEG):
		ref.ref[ha.RKN] = thetaKnee
		ref.ref[ha.RHP] = thetaHip
		ref.ref[ha.RAP] = thetaAnkle 

	r.put(ref)
	simSleep(0.2)
			
	####################################################################		
	numSteps = 26
	
	#HIP ANKLE ROLL aka Lean
	stepInit = -LEAN_ANGLE
	stepSize = 0.01
	
	#Front Leg
	xFrontLegInit = WALK_DOWN_LEG_X
	#Back Leg
	xBackLegInit = SQUAT_DOWN_X
	legStepSize = 2
	
	for i in range(numSteps):
		step = stepInit + (stepSize * i)
		# Lean
		if(leg == LEFT_LEG):
			ref.ref[ha.RAR] = step  
			ref.ref[ha.LAR] = step 
			ref.ref[ha.RHR] = -step 
			ref.ref[ha.LHR] = -step 
		if(leg == RIGHT_LEG):
			ref.ref[ha.RAR] = -step  
			ref.ref[ha.LAR] = -step 
			ref.ref[ha.RHR] = step 
			ref.ref[ha.LHR] = step 

		
		#forward
		#front leg
		xFrontLegStep = xFrontLegInit + (legStepSize * i)
		frontLegThetaHip, frontLegThetaKnee, frontLegThetaAnkle = legsInverseKinematics(xFrontLegStep, SQUAT_DOWN_Y, UPPER_LEG, LOWER_LEG)
		if (leg == LEFT_LEG):
			ref.ref[ha.LKN] = frontLegThetaKnee
			ref.ref[ha.LHP] = frontLegThetaHip
			ref.ref[ha.LAP] = frontLegThetaAnkle 
		if (leg == RIGHT_LEG):
			ref.ref[ha.RKN] = frontLegThetaKnee
			ref.ref[ha.RHP] = frontLegThetaHip
			ref.ref[ha.RAP] = frontLegThetaAnkle 
		
		#back leg
		xBackLegtStep = xBackLegInit + (legStepSize * i)
		backLegThetaHip, backLegThetaKnee, backLegThetaAnkle = legsInverseKinematics(xBackLegtStep, WALK_DOWN_LEG_Y, UPPER_LEG, LOWER_LEG)
		if (leg == LEFT_LEG):
			ref.ref[ha.RKN] = backLegThetaKnee	
			ref.ref[ha.RHP] = backLegThetaHip	
			ref.ref[ha.RAP] = backLegThetaAnkle
		if (leg == RIGHT_LEG):
			ref.ref[ha.LKN] = backLegThetaKnee	
			ref.ref[ha.LHP] = backLegThetaHip	
			ref.ref[ha.LAP] = backLegThetaAnkle
	
		r.put(ref)
		simSleep(.05) 

		print "Lean Left : Current Angle = ", step, "xFrontLegStep = ", xFrontLegStep, "xBackLegtStep = ", xBackLegtStep
	
	
	# Write to the feed-forward channel
	r.put(ref)
	simSleep(0.1)

	return

def walk(numSteps):
	for i in range(numSteps):
		print "**************** STEP " , i + 1, " ****************" 
		print state.time , "Starting Step for left leg"
		stepLeg(LEFT_LEG)

		print state.time , "Starting Step for Right leg"
		stepLeg(RIGHT_LEG)
#########################################################################
# Rotation matrices
def rX(theta):
	R = np.identity(4)
	R[1,1] = np.cos(theta)
	R[1,2] = np.sin(theta) * -1.0
	R[2,1] = np.sin(theta)
	R[2,2] = np.cos(theta)
	
	return R

def rY(theta):
	R = np.identity(4)
	R[0,0] = np.cos(theta)
	R[0,2] = np.sin(theta)
	R[2,0] = np.sin(theta) * -1.0
	R[2,2] = np.cos(theta)

	return R

def rZ(theta):
	R = np.identity(4)
	R[0,0] = np.cos(theta)
	R[0,1] = np.sin(theta) * -1.0
	R[1,0] = np.sin(theta)
	R[1,1] = np.cos(theta)

	return R


def getFK(whichArm, thetas): #theta0, theta1, theta2, ....
	
	#shoulder Pitch
	T1 = np.identity(4)
	if(whichArm == LEFT_ARM):
		T1[1,3] = LENGTH_BASE_TO_SHOULDER
	elif(whichArm == RIGHT_ARM):
		T1[1,3] = LENGTH_BASE_TO_SHOULDER * -1.0
	Q1 = np.dot(rY(thetas[0,0]), T1)
	
	#shoulder Roll
	T2 = np.identity(4)
	#T2[1,3] = LENGTH_BASE_TO_SHOULDER
	Q2 = np.dot(rX(thetas[1,0]), T2)
	
	#shoulder Yaw
	T3 = np.identity(4)
	#T3[1,3] = LENGTH_BASE_TO_SHOULDER
	Q3 = np.dot(rZ(thetas[2,0]), T3)
	
	#elbow Pitch
	T4 = np.identity(4)
	T4[2,3] = LENGTH_SHOULDER_TO_ELBOW * -1.0
	Q4 = np.dot(rY(thetas[3,0]), T4)

	#wrist Yaw
	T5 = np.identity(4)
	T5[2,3] = (LENGTH_ELBOW_TO_WRIST + LENGTH_WRIST_TO_FINGER) * -1.0
	Q5 = np.dot(rZ(thetas[4,0]), T5)

	#wrist roll
	T6 = np.identity(4)
	Q6 = np.dot(rX(thetas[5,0]), T6)
	
	#not using pitch
	
	Qend = np.dot(Q1, Q2)
	Qend = np.dot(Qend, Q3)
	Qend = np.dot(Qend, Q4)
	Qend = np.dot(Qend, Q5)
	Qend = np.dot(Qend, Q6)
	
	endEff = np.array([[round(Qend[0,3],3)], [round(Qend[1,3],3)], [round(Qend[2,3],3)]])
	
	return endEff

def getJacobian(whichArm, thetas, deltaTheta):
	Jac = np.zeros((3,6))
	for i in range((np.shape(Jac))[0]): #3
		for j in range((np.shape(Jac))[1]): #6
			#print "********* ", i,j
			newThetas = np.copy(thetas)
			newThetas[j] = thetas[j] + deltaTheta		
			newEndEff = getFK(whichArm, newThetas)
			#print newEndEff
			Jac[i,j] = (newEndEff[i,0] )/ deltaTheta
	return Jac

def getDistance(endEff, goal):
	m = math.sqrt(math.pow(endEff[0] - goal[0],2) + math.pow(endEff[1] - goal[1],2) + math.pow(endEff[2] - goal[2],2))
	return m

def getNext(endEff, goal, eStep, dist):
	dx = (goal[0] - endEff[0]) * eStep / dist
	dy = (goal[1] - endEff[1]) * eStep / dist
	dz = (goal[2] - endEff[2]) * eStep / dist
	
	deltaEndEff = np.array([[round(dx,3)], [round(dy,3)], [round(dz,3)]])
	
	return deltaEndEff
	
def setArmThetas(leftThetas, rightThetas):

	ref.ref[ha.LSP] = leftThetas[0]
	ref.ref[ha.LSR] = leftThetas[1]
	ref.ref[ha.LSY] = leftThetas[2]
	ref.ref[ha.LEB] = leftThetas[3]
	ref.ref[ha.LWY] = leftThetas[4]
	ref.ref[ha.LWR] = leftThetas[5]

	ref.ref[ha.RSP] = rightThetas[0]
	ref.ref[ha.RSR] = rightThetas[1]
	ref.ref[ha.RSY] = rightThetas[2]
	ref.ref[ha.REB] = rightThetas[3]
	ref.ref[ha.RWY] = rightThetas[4]
	ref.ref[ha.RWR] = rightThetas[5]

	r.put(ref)

def moveArm(whichArm, thetaInit, goal, deltaTheta, eStep, error):

	if(whichArm == LEFT_ARM):
		strArm = "LEFT  ARM "
	else:
		strArm = "RIGHT ARM "	
	endEff = getFK(whichArm, thetaInit)
	thetas = np.copy(thetaInit)
	dist = getDistance(endEff, goal)
	orig_dist = dist
	print strArm, "Goal    Position : ", goal.transpose()
	#print "Current Theta    : ", thetas.transpose()
	print strArm, "Current Position : ", endEff.transpose()
	print strArm, "Error Allowed : ", error, "Distance : ", dist
	counter = 0
	while(dist > error):
		print "\nstrArm, Goal    Position : ", goal.transpose()
		Jac = getJacobian(whichArm, thetas, deltaTheta)
		invJac = np.linalg.pinv(Jac)

		deltaEndEff = getNext(endEff, goal, eStep, orig_dist)
		#print "Increm  Position : ", deltaEndEff.transpose()

		changeTheta = np.dot(invJac, deltaEndEff)
		#print "Change Theta     : ", changeTheta.transpose()

		thetas = np.add(thetas, changeTheta)
		#print "New Thetas       : ", thetas.transpose()

		endEff = getFK(whichArm, thetas)
		print strArm, "New Position : ", endEff.transpose()	
		
		dist = getDistance(endEff, goal)		
		print strArm, "Error Allowed : ", error, "Distance : ", dist
		counter = counter + 1
	
	#print "Counter : ", counter
	return thetas
	
#######################################################################
	
def simSleep(sec):
	tick = state.time;
	dt = 0;
	while(dt <= sec):
		s.get(state, wait=False, last=True)
		dt = state.time - tick;
	return

print state.time , "Crouch"	
squat()

print state.time , "Lean right"
walkLeanRight()

print state.time, "Starting Walk"
walk(11)

newRightThetas = moveArm(RIGHT_ARM, rightThetaInit, RIGHT_GOAL_TOP_STRAIGHT, DELTA_THETA, ENDEFF_STEP, ERROR)
setArmThetas(newLeftThetas, newRightThetas)
print "******* Move to Top Right *******"

# Close the connection to the channels
r.close()
s.close()
