# Quadcopter simulator

#*****************************************************************************#
# Imports
#*****************************************************************************#
import math
from copy import deepcopy

#*****************************************************************************#
# Classes
#*****************************************************************************#


#*****************************************************************************#
# Variables and objects
#*****************************************************************************#
# All units are SI base units

#TODO: Make sure this is accurate relative to quadcopter I.R.L.!!!
# * Arm holding motor 0 points in +y, motor 1 arm in -y direction, motor 2 arm
#   in +x direction, motor 3 arm in -x direction
# * Yaw measured...
# * Pitch measured relative to x and z axes
# * Roll measured relative to y and z axes

# Output file
outputFile = open('output','w')

# Overall simulation timings
time = 0.0				# Units of seconds
interval = 0.0001
simulationLength = 50.0
currentProgress = -1.0
previousProgress = 0

# Performance measures
errorRMS = 0

# Quadcopter physical characteristics
# TODO: Measure these precisely!
quadcopterMass = 1
quadcopterArmLength = 0.3
quadcopterMomentOfInertia = [0.05,0.05,1]	# Corresponds to leading diagonal of inertia matrix. First two elements must be the same.

# Quadcopter motion
quadcopterTorque = [0.0,0.0,0.0]	# [about +x, about +y, about +z]
quadcopterAngularAcceleration = [0.0,0.0,0.0]
quadcopterAngularVelocity = [0.0,0.0,0.0]
quadcopterOrientation = [0.0,0.0,-1.0]	# [yaw,pitch,roll]

# Sensor characteristics
sensorReadings = [deepcopy(quadcopterOrientation)]
sensorReadingTimes = [0.0]
sensorUpdatePeriod = 0.15#.2
motorCommandSent = 1

# Motor characteristics
motorControllerValue = [0.0,0.0,0.0,0.0]	# [M0,M1,M2,M3]
motorForce = [0.0,0.0,0.0,0.0]

# Control variables, e.g. PID values etc.
defaultMotorControllerValue = 40
PIDConstants = [20.0,0.0,8.0]
PIDTerms = [0.0,0.0,0.0]
profileStartTime = -1.0

#*****************************************************************************#		
# Define functions
#*****************************************************************************#
# Calculates torque on the quadcopter based on torques from the motors and
# the simulated environment, e.g. wind, pull from tethers, etc.
def calculateQuadcopterTorque(motorForce,quadcopterTorque):
	# Convert forces into torques
	quadcopterTorque[0] = motorForce[0]*quadcopterArmLength - motorForce[1]*quadcopterArmLength
	quadcopterTorque[1] = motorForce[2]*quadcopterArmLength - motorForce[3]*quadcopterArmLength
	quadcopterTorque[2] = 0 # TODO: Work out what this should be
	
	return

# Calculates Forces produced by motors based on ESC values
def calculateMotorForce(motorValue,motorForce):
	for i in range(len(motorForce)):
		if motorValue[i] <= 33:
			motorForce[i] = 0
		elif motorValue[i] >= 60:
			motorForce[i] = 1.76
		else:
			motorForce[i] = (motorValue[i] - 33)*0.065
	return

# Simulates receiving a reading from the sensor
def getSensorReading(quadcopterOrientation):
	# TODO: Add sensor noise!
	return quadcopterOrientation

# Generic PID controller
def incrementPID(error,previousError,PIDConstants,PIDTerms,timeElapsed):
	PIDTerms[0] = PIDConstants[0]*error
	PIDTerms[1] += PIDConstants[1]*error*timeElapsed
	PIDTerms[2] = PIDConstants[2]*(error-previousError)/timeElapsed
	
	outputValue = PIDTerms[0] + PIDTerms[1] + PIDTerms[2]
	
	return outputValue

# Simulates sending a command to the ESC - uncomment required version
# Runs PID loop directly on angle error
'''
def calculateMotorCommand(sensorReadings,sensorReadingTimes,PIDConstants,PIDTerms):
	#PID calculation!
	PIDOutput = incrementPID(0.0-sensorReadings[-1][2],0.0-sensorReadings[-2][2],PIDConstants,PIDTerms,sensorReadingTimes[-1]-sensorReadingTimes[-2])
	
	return [45+PIDOutput,45-PIDOutput,0.0,0.0]
'''	
# Defines velocity profile, PID loop is run on error in velocity
def calculateMotorCommand(sensorReadings,sensorReadingTimes,PIDConstants,PIDTerms):
	if(profileStartTime < 0) profileStartTime = sensorReadingTimes[-1]
	
	return [0.0,0.0,0.0,0.0]

#*****************************************************************************#
# Main loop
#*****************************************************************************#
while time < simulationLength:
	
	# increment time
	time += interval
	
	# Calculate forces produced by motors
	calculateMotorForce(motorControllerValue,motorForce)
	
	# Calculate non-angular forces on quadcopter
	# calculateQuadcopterForces()
	
	# Integrate to produce resultant position and velocity
	# calculateMotion()
	
	# Perform velocity Verlet integration of torque to get angular velocity and
	# position
	quadcopterOrientation[2] += interval*(quadcopterAngularVelocity[0] + 0.5*quadcopterAngularAcceleration[0]*interval)
	if(quadcopterOrientation[2] > math.pi):
		quadcopterOrientation[2] -= 2*math.pi
	if(quadcopterOrientation[2] < -math.pi):
		quadcopterOrientation[2] += 2*math.pi
	quadcopterAngularVelocity[0] += 0.5*quadcopterAngularAcceleration[0]*interval
	
	# Calculate torque on quadcopter
	calculateQuadcopterTorque(motorForce,quadcopterTorque)
	
	quadcopterAngularAcceleration[0] = quadcopterTorque[0]/quadcopterMomentOfInertia[0]
	quadcopterAngularAcceleration[1] = quadcopterTorque[1]/quadcopterMomentOfInertia[1]
	quadcopterAngularAcceleration[2] = quadcopterTorque[2]/quadcopterMomentOfInertia[2]
	
	quadcopterAngularVelocity[0] += 0.5*quadcopterAngularAcceleration[0]*interval
	# End of torque integration

	# Sensor readings are received at well defined intervals
	if ((time - sensorReadingTimes[-1]) >= sensorUpdatePeriod):
		sensorReadings.append(deepcopy(getSensorReading(quadcopterOrientation)))
		sensorReadingTimes.append(time)
		motorCommandSent = 0
		
		# Write readings to file
		outputString = repr(time)[:6] + ',' + repr(sensorReadings[-1][2]) + '\n'
		outputFile.write(outputString)
	
	# Motor commands sent out ~1ms after the sensor makes a reading,
	# however the torque produced by the motor may vary in a less quantised
	# manner. TODO: Implement torque variation as a function of time
	if (((time - sensorReadingTimes[-1]) >= 0.001) and (motorCommandSent == 0)):
		motorControllerValue = calculateMotorCommand(sensorReadings,sensorReadingTimes,PIDConstants,PIDTerms)
		motorCommandSent = 1
		#print(time,PIDTerms[2])
	
	# Print current progress
	currentProgress = math.floor(100*time/simulationLength)
	if(currentProgress != previousProgress):
		print(repr(currentProgress)[:4]+'%',"complete")
		previousProgress = currentProgress
		
	errorRMS += (quadcopterOrientation[2]-0.0)**2*interval
	
print("RMS error is:",errorRMS**0.5)

