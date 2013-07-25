import serial
from serial.tools import list_ports
import time
import struct

###############################################################################
#                                                                             #
#                           Function definitions                              #
#                                                                             #
###############################################################################

def alignment(port):
	x = 'h'
	while x != '\n':
		x = port.read()

###############################################################################
#                                                                             #
#                           Start of main code                                #
#                                                                             #
###############################################################################

#Find list of available ports
print("Ports which appear to have a USB device attached:\n")

portList = list(list_ports.grep('USB'))
for i in enumerate(portList): print("Device " + str(i[0]+1) + ": " + "\nHardware info: " + i[1][2] + "\n")

#Ask user which one they want to use
print("Enter the number of the device you wish to connect to: ")
portNum = int(raw_input('>'))

#Open serial port
newPort = serial.Serial(portList[portNum-1][0],115200)
print("Opened " + newPort.portstr)

#TODO: Check if arduino has sent initialisation character
time.sleep(1)
#TODO: exit program if arduino does not send initialisation character after a reasonable wait
#Send character to tell arduino to start sending data
newPort.flushInput()
newPort.write('h')
time.sleep(1)

newPort.flushInput()
b = [0]*47
#Align with starting position of data packet
alignment(newPort)
x = newPort.readline()
print(x)

#for i in range(10):
#	newPort.readinto(b)
#	print b
#	if ((b[0] == '$') and (b[45] == '\r') and (b[46] == '\n')):
#		print "Packets aligned properly! :)"
#		
#	else:
#		print "Packets not properly aligned :("
#		alignment(newPort)
#		continue
#	#convert appropriate sections of b to a stirng
#	str_b = ''.join(str(e) for e in b[1:5])
#	print struct.unpack('>I',str_b)
#	str_b = ''.join(str(e) for e in b[5:9])
#	print struct.unpack('>f',str_b)
#	str_b = ''.join(str(e) for e in b[9:13])
#	print struct.unpack('>f',str_b)
#	str_b = ''.join(str(e) for e in b[13:17])
#	print struct.unpack('>f',str_b)
#	str_b = ''.join(str(e) for e in b[17:21])
#	print struct.unpack('>f',str_b)
#	str_b = ''.join(str(e) for e in b[21:25])
#	print struct.unpack('>f',str_b)
#	str_b = ''.join(str(e) for e in b[25:29])
#	print struct.unpack('>f',str_b)
#	str_b = ''.join(str(e) for e in b[29:33])
#	print struct.unpack('>f',str_b)
#	str_b = ''.join(str(e) for e in b[33:37])
#	print struct.unpack('>f',str_b)
#	str_b = ''.join(str(e) for e in b[37:41])
#	print struct.unpack('>f',str_b)
#	str_b = ''.join(str(e) for e in b[41:45])
#	print struct.unpack('>f',str_b)
newPort.close()
