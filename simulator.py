# Quadcopter simulator

#*****************************************************************************#
# Imports
#*****************************************************************************#
import math
from copy import deepcopy
from matplotlib import pyplot as plt

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
time = [0.0]				# Units of seconds
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
quadcopterOrientation = [[0.0,0.0,-1.0]]	# [yaw,pitch,roll]

# Sensor characteristics
sensorReadings = [deepcopy(quadcopterOrientation[-1])]
sensorReadingTimes = [0.0]
sensorUpdatePeriod = 0.01#.2
motorCommandSent = 1

# Motor characteristics
motorControllerValue = [0.0,0.0,0.0,0.0]	# [M0,M1,M2,M3]
motorForce = [0.0,0.0,0.0,0.0]
maxMotorAcceleration = 1.1

# Control variables, e.g. PID values etc.
maxDifference = 5
defaultMotorControllerValue = (60.0+33.0)/2
PIDConstants = [20.0,0.0,10.0]
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
def calculateMotorForce(timeList,motorValue,motorForce):
	desiredForce = 0
	for i in range(len(motorForce)):
		if motorValue[i] <= 33:
			desiredForce = 0.0
		elif motorValue[i] >= 60:
			desiredForce = 1.76
		else:
			desiredForce = (motorValue[i] - 33.0)*0.065
		
		if (desiredForce-motorForce[i])/(timeList[-1]-timeList[-2]) > maxMotorAcceleration:
			motorForce[i] = motorForce[i] + maxMotorAcceleration*(timeList[-1]-timeList[-2])
		elif (desiredForce-motorForce[i])/(timeList[-1]-timeList[-2]) < -maxMotorAcceleration:
			motorForce[i] = motorForce[i] - maxMotorAcceleration*(timeList[-1]-timeList[-2])
		else:
			motorForce[i] = desiredForce
			
		
	return

# Simulates receiving a reading from the sensor
def getSensorReading(quadcopterOrientation):
	# TODO: Add sensor noise!
	return quadcopterOrientation

def verletStep(value,derivative,secondDerivative,calculateSecondDerivative,stepSize):
	nextValue = value + stepSize*(derivative + 0.5*secondDerivative*stepSize)
	nextDerivative = derivative + 0.5*secondDerivative*stepSize
	
	# Calculate torque on quadcopter
#	calculateSecondDerivative(motorForce,quadcopterTorque)
	
#	quadcopterAngularAcceleration[0] = quadcopterTorque[0]/quadcopterMomentOfInertia[0]
#	quadcopterAngularAcceleration[1] = quadcopterTorque[1]/quadcopterMomentOfInertia[1]
#	quadcopterAngularAcceleration[2] = quadcopterTorque[2]/quadcopterMomentOfInertia[2]
	
#	quadcopterAngularVelocity[0] += 0.5*quadcopterAngularAcceleration[0]*interval

# Generic PID controller
def incrementPID(error,previousError,PIDConstants,PIDTerms,timeElapsed):
	PIDTerms[0] = PIDConstants[0]*error
	PIDTerms[1] += PIDConstants[1]*error*timeElapsed
	PIDTerms[2] = PIDConstants[2]*(error-previousError)/timeElapsed
	
	outputValue = PIDTerms[0] + PIDTerms[1] + PIDTerms[2]
	
	return outputValue

# Simulates sending a command to the ESC - uncomment required version
# Runs PID loop directly on angle error
def calculateMotorCommand(sensorReadings,sensorReadingTimes,PIDConstants,PIDTerms):
	#PID calculation!
	PIDOutput = incrementPID(0.0-sensorReadings[-1][2],0.0-sensorReadings[-2][2],PIDConstants,PIDTerms,sensorReadingTimes[-1]-sensorReadingTimes[-2])
	
	if(2*PIDOutput > maxDifference):
		PIDOutput = maxDifference/2
	return [defaultMotorControllerValue+PIDOutput,defaultMotorControllerValue-PIDOutput,0.0,0.0]

'''
# Defines velocity profile, PID loop is run on error in velocity
def calculateMotorCommand(sensorReadings,sensorReadingTimes,PIDConstants,PIDTerms):
	if(profileStartTime < 0): profileStartTime = sensorReadingTimes[-1]
	
	return [0.0,0.0,0.0,0.0]
'''
#*****************************************************************************#
# Main loop
#*****************************************************************************#
while time[-1] < simulationLength:
	
	# increment time
	time.append(time[-1]+interval)
	
	# Calculate forces produced by motors
	calculateMotorForce(time,motorControllerValue,motorForce)
	
	# Calculate non-angular forces on quadcopter
	# calculateQuadcopterForces()
	
	# Integrate to produce resultant position and velocity
	# calculateMotion()
	
	# Perform velocity Verlet integration of torque to get angular velocity and
	# position
	newOrientation = [0,0,0]
	newOrientation[2] = quadcopterOrientation[-1][2] + interval*(quadcopterAngularVelocity[0] + 0.5*quadcopterAngularAcceleration[0]*interval)
	if(newOrientation[2] > math.pi):
		newOrientation[2] -= 2*math.pi
	if(newOrientation[2] < -math.pi):
		newOrientation[2] += 2*math.pi
	quadcopterOrientation.append(newOrientation)
	
	quadcopterAngularVelocity[0] += 0.5*quadcopterAngularAcceleration[0]*interval
	
	# Calculate torque on quadcopter
	calculateQuadcopterTorque(motorForce,quadcopterTorque)
	
	quadcopterAngularAcceleration[0] = quadcopterTorque[0]/quadcopterMomentOfInertia[0]
	quadcopterAngularAcceleration[1] = quadcopterTorque[1]/quadcopterMomentOfInertia[1]
	quadcopterAngularAcceleration[2] = quadcopterTorque[2]/quadcopterMomentOfInertia[2]
	
	quadcopterAngularVelocity[0] += 0.5*quadcopterAngularAcceleration[0]*interval
	# End of torque integration

	# Sensor readings are received at well defined intervals
	if ((time[-1] - sensorReadingTimes[-1]) >= sensorUpdatePeriod):
		sensorReadings.append(deepcopy(getSensorReading(quadcopterOrientation[-1])))
		sensorReadingTimes.append(time[-1])
		motorCommandSent = 0
		
		# Write readings to file
		outputString = repr(time[-1])[:6] + ',' + repr(sensorReadings[-1][2]) + '\n'
		outputFile.write(outputString)
	
	# Motor commands sent out ~1ms after the sensor makes a reading,
	# however the torque produced by the motor may vary in a less quantised
	# manner. TODO: Implement torque variation as a function of time
	if (((time[-1] - sensorReadingTimes[-1]) >= 0.001) and (motorCommandSent == 0)):
		motorControllerValue = calculateMotorCommand(sensorReadings,sensorReadingTimes,PIDConstants,PIDTerms)
		motorCommandSent = 1
		#print(time,PIDTerms[2])
	
	# Print current progress
	currentProgress = math.floor(100*time[-1]/simulationLength)
	if(currentProgress != previousProgress):
		print(str(currentProgress)[:4]+"% complete")
		previousProgress = currentProgress
		
	errorRMS += (quadcopterOrientation[-1][2]-0.0)**2*interval
	
print("RMS error is:"+str(errorRMS**0.5))

plt.plot([x for x in time],[x[2] for x in quadcopterOrientation])
plt.show()
