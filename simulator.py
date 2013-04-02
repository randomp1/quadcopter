# Quadcopter simulator

#*****************************************************************************#
# Imports
#*****************************************************************************#
import math

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

# Overall simulation timings
time = 0.0				# Units of seconds
interval = 0.0001
simulationLength = 20.0

# Sensor characteristics
lastSensorReading = 0.0
sensorUpdatePeriod = 0.1
sensorOrientation = [0.0,0.0,0.0]
previousSensorOrientation = [0.0,0.0,0.0]

# Quadcopter physical characteristics
# TODO: Measure these precisely!
quadcopterMass = 1
quadcopterArmLength = 0.3
quadcopterMomentOfInertia = 0.05

# Quadcopter motion
quadcopterTorque = [0.0,0.0,0.0]	# [yaw,pitch,roll]
quadcopterAngularAcceleration = [0.0,0.0,0.0]
quadcopterAngularVelocity = [0.0,0.0,0.0]
quadcopterOrientation = [0.0,0.0,-1.0]

# Motor characteristics
motorControllerValue = [0.0,0.0,0.0,0.0]	# [M0,M1,M2,M3]
motorForce = [0.0,0.0,0.0,0.0]

# Control variables, e.g. PID values etc.
defaultMotorControllerValue = 40
PIDConstants = [1.0,0.0,0.0]
PIDTerms = [0.0,0.0,0.0]

#*****************************************************************************#		
# Define functions
#*****************************************************************************#
# Calculates torque on the quadcopter based on torques from the motors and
# the simulated environment, e.g. wind, pull from tethers, etc.
def calculateQuadcopterTorque(motorForce,quadcopterTorque):
	# Convert forces into torques
	quadcopterTorque[0] = 0 # God knows what this ought to be :S...
	quadcopterTorque[1] = motorForce[2]*quadcopterArmLength - motorForce[3]*quadcopterArmLength
	quadcopterTorque[2] = motorForce[0]*quadcopterArmLength - motorForce[1]*quadcopterArmLength
	
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
def getSensorReading(quadcopterOrientation,sensorOrientation):
	for i in range(len(quadcopterOrientation)):
		# TODO: Add sensor noise!
		sensorOrientation[i] = quadcopterOrientation[i]
	return

# Simulates sending a command to the ESC
def sendMotorCommand(sensorOrientation,motorValue,PIDConstants,PIDTerms,timeSincePreviousMotorCommand):
	#PID calculation!
	PIDTerms[0] = PIDConstants[0]*(0-sensorOrientation[2])
	PIDTerms[1] += PIDConstants[1]*(0-sensorOrientation[2])*timeSincePreviousMotorCommand
	PIDTerms[2] = PIDConstants[2]*((0-sensorOrientation[2])-previousSensorOrientation[2])/timeSincePreviousMotorCommand
	
	motorValue[0] = 45 + PIDTerms[0] + PIDTerms[1] + PIDTerms[2]
	motorValue[1] = 45 - (PIDTerms[0] + PIDTerms[1] + PIDTerms[2])
	return

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
	
	quadcopterOrientation[2] += interval*(quadcopterAngularVelocity[2] + 0.5*quadcopterAngularAcceleration[2]*interval)
	if(quadcopterOrientation[2] > math.pi):
		quadcopterOrientation[2] -= math.pi
	if(quadcopterOrientation[2] < -math.pi):
		quadcopterOrientation[2] += math.pi
	quadcopterAngularVelocity[2] += 0.5*quadcopterAngularAcceleration[2]*interval
	
	# Calculate torque on quadcopter
	calculateQuadcopterTorque(motorForce,quadcopterTorque)
	
	quadcopterAngularAcceleration[0] = 0.0 # TODO: work out what this is!
	quadcopterAngularAcceleration[1] = 0.0 # TODO: work out what this is!
	quadcopterAngularAcceleration[2] = quadcopterTorque[2]/quadcopterMomentOfInertia
	
	quadcopterAngularVelocity[2] += 0.5*quadcopterAngularAcceleration[2]*interval

	# Sensor readings are received at well defined intervals
	if ((time - lastSensorReading) >= sensorUpdatePeriod):
		getSensorReading(quadcopterOrientation,sensorOrientation)
		lastSensorReading = time
		print(repr(time)[:6],sensorOrientation)
	
	# Motor commands sent out ~1ms after the sensor makes a reading times,
	# however the torque produced by the motor may vary in a less quantised
	# manner. TODO: Implement torque variation as a function of time
	if ((time - lastSensorReading) >= 0.001):
		sendMotorCommand(sensorOrientation,motorControllerValue,PIDConstants,PIDTerms,(time-lastSensorReading))
		


