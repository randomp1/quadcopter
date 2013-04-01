# Quadcopter simulator

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
sensorUpdatePeriod = 0.05

# Quadcopter physical characteristics
quadcopterMass = 1;
quadcopterArmLength = 0.3;

# Quadcopter motion
quadcopterOrientation = [0.0,0.0,0.0]	# [yaw,pitch,roll]
quadcopterTorque = [0.0,0.0,0.0]

# Motor characteristics
motorControllerValue = [0.0,0.0,0.0,0.0]	# [M0,M1,M2,M3]
motorForce = [0.0,0.0,0.0,0.0]

#*****************************************************************************#		
# Define functions
#*****************************************************************************#
# Calculates torque on the quadcopter based on torques from the motors and
# the simulated environment, e.g. wind, pull from tethers, etc.
def calculateQuadcopterTorque(motorForce,quadcopterTorque):
	# Convert forces into torques
	quadcopterTorque[0] = 0 # God knows what this ought to be :S...
	quadcopterTorque[1] = motorForce[0]*quadcopterArmLength - motorForce[1]*quadcopterArmLength
	quadcopterTorque[2] = motorForce[2]*quadcopterArmLength - motorForce[3]*quadcopterArmLength
	
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
	
def calculateMotion():
	pass
	return

# Simulates sending a command to the ESC
def sendMotorCommand():
	pass
	return

# Simulates receiving a reading from the sensor
def getSensorReading(quadcopterOrientation,sensorOrientation):
	for i in range(len(quadcopterOrientation)):
		# TODO: Add sensor noise!
		sensorOrientation[i] = quadcopterOrientation[i]
	return

#*****************************************************************************#
# Main loop
#*****************************************************************************#
while time < simulationLength:

	# increment time
	time += interval
	
	# Calculate forces produced by motors
	calculateMotorForce(motorControllerValue,motorForce)
	
	# Calculate torque on quadcopter
	calculateQuadcopterTorque(motorForce,quadcopterTorque)
	
	# Integrate to produce resultant motion
	calculateOrientation(quadcopterTorque)
	

	# Sensor readings are received at well defined intervals
	if ((time - lastSensorReading) >= sensorUpdatePeriod):
		getSensorReading()
	
	# Motor commands sent out ~1ms after the sensor makes a reading times,
	# however the torque produced by the motor may vary in a less quantised
	# manner.
	if ((time - lastSensorReading) >= 0.001):
		sendMotorCommand()


