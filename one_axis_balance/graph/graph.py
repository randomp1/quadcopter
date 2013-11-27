import serial
from serial.tools import list_ports
import time
import struct
from matplotlib.pyplot import subplots,close
from numpy import polyfit
import threading
import sys
import select

###############################################################################
#                                                                             #
#                           Function definitions                              #
#                                                                             #
###############################################################################

def alignment(port):
	x = 'h'
	while x != '\n':
		x = port.read()
		
# Define function to fit
def fitFunction(args):
	optimiseThis = 0
	for x in fitData:
		optimiseThis += ((args[0]*x[0]**2+args[1]*x[0]+args[2]) - x[1])**2
	
	return optimiseThis
	
# Fits parabola of form y=ax^2+bx+c to data
def fitParabola(xVals,yVals):
	assert(len(xVals) == len(yVals))
	
	#Scale our input values such that -1 < y < 1 and -1 < x < 1
	xTrans = (max(xVals)+min(xVals))/2.0
	yTrans = (max(yVals)+min(yVals))/2.0
	xScale = (max(xVals)-min(xVals))/2.0
	yScale = (max(yVals)-min(yVals))/2.0
	
	# Calculate different sums: x4 = sum(x[i]^4) for example
	x4 = 0.0
	x3 = 0.0
	x2 = 0.0
	x = 0.0
	y = 0.0
	xy = 0.0
	x2y = 0.0
	
	for i in range(len(xVals)):
		x4 = x4 + pow((xVals[i]-xTrans)/xScale,4)
		x3 = x3 + pow((xVals[i]-xTrans)/xScale,3)
		x2 = x3 + pow((xVals[i]-xTrans)/xScale,2)
		x =  x + (xVals[i]-xTrans)/xScale
		y = y + (yVals[i]-yTrans)/yScale
		xy = xy + (xVals[i]-xTrans)/xScale * (yVals[i]-yTrans)/yScale
		x2y = x2y + pow((xVals[i]-xTrans)/xScale,2)*(yVals[i]-yTrans)/yScale
		#print(xVals[i])
		print((xVals[i]-xTrans)/xScale)
	
	print([x4,x3,x2,x,y,xy,x2y])
	# Calculate the coefficients
	n = len(xVals)
	aScaled = ((x*y/n-xy)*(x3-x*x2/n)-(x2-pow(x,2)/n)*(x2*y/n-x2y))/((x4-pow(x2,2)/n)*(x2-pow(x,2)/n)-(x3-x*x2/n)*(x3-x*x2/n))
	bScaled = -(aScaled*(x3-x*x2)+x*y-xy)/(x2-pow(x,2))
	cScaled = -aScaled*x2-bScaled*x+y
	
	a = yScale*aScaled/pow(xScale,2)
	b = yScale/xScale * (bScaled-2*xTrans*a)
	c = yTrans + cScaled - a*pow(xTrans,2) - b*xTrans
	
	return (a,b,c)

# Handles command line input by forwarding it to arduino
def consoleInput(serialPort,terminate):
	while(terminate[0] == False):
	#	command = raw_input('')
	#	serialPort.write(command)
	
	# If there's input ready, do something, else do something
	# else. Note timeout is zero so select won't block at all.
		while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
			line = sys.stdin.readline()[:-1] # Read line and remove trailing newline character
			if line:
				serialPort.write(line) # Print to the terminal!
			else: # an empty line means stdin has been closed
				print('eof')
				exit(0)
		else:
			time.sleep(0.1)
	#print("consoleInputThread returning")

# Reads input from the arduino
def arduinoInput(serialPort,data,terminate):
	# Main loop reading and handling data
	b = [0]*47
	aligned = 0
	misAligned = 0
	fitData = []
	bufferMax = 0
	while(terminate[0] == False):#aligned < 1000):
		if(serialPort.inWaiting() >= 47):
			serialPort.readinto(b)
			#print(b)
			if ((b[0] == '$') and (b[45] == '\r') and (b[46] == '\n')):
				#print("Packets aligned properly! :)")
				aligned += 1
			else:
				print("Packets not properly aligned :(")
				misAligned +=1
				alignment(serialPort)
				continue
			dataPoint = []
			#convert appropriate sections of b to a stirng
			str_b = ''.join(str(e) for e in b[1:5])
			dataPoint.append(struct.unpack('>I',str_b)[0])
			str_b = ''.join(str(e) for e in b[5:9])
			dataPoint.append(struct.unpack('>f',str_b)[0])
			str_b = ''.join(str(e) for e in b[9:13])
			dataPoint.append(struct.unpack('>f',str_b)[0])
			str_b = ''.join(str(e) for e in b[13:17])
			dataPoint.append(struct.unpack('>f',str_b)[0])
			str_b = ''.join(str(e) for e in b[17:21])
			dataPoint.append(struct.unpack('>f',str_b)[0])
			str_b = ''.join(str(e) for e in b[21:25])
			dataPoint.append(struct.unpack('>f',str_b)[0])
			str_b = ''.join(str(e) for e in b[25:29])
			dataPoint.append(struct.unpack('>f',str_b)[0])
			str_b = ''.join(str(e) for e in b[29:33])
			dataPoint.append(struct.unpack('>f',str_b)[0])
			str_b = ''.join(str(e) for e in b[33:37])
			dataPoint.append(struct.unpack('>f',str_b)[0])
			str_b = ''.join(str(e) for e in b[37:41])
			dataPoint.append(struct.unpack('>f',str_b)[0])
			str_b = ''.join(str(e) for e in b[41:45])
			dataPoint.append(struct.unpack('>f',str_b)[0])
		
			#print(data)
			data.append(dataPoint)

			fitData.append([dataPoint[0],dataPoint[3]])
			if(len(fitData) == 21):
				fitData.pop(0)
				#result = polyfit([x[0] for x in fitData],[x[1] for x in fitData],2)
				#result2 = fitParabola([x[0] for x in fitData],[x[1] for x in fitData])
				#print("Old: " + str(result) + "\nNew: " + str(result2))
			
			if(serialPort.inWaiting() > bufferMax): bufferMax = serialPort.inWaiting()
			#print("Buffer: " + str(serialPort.inWaiting()) + " Bytes, highest: " + str(bufferMax) + " Bytes | " + str(misAligned) + "/" + str(aligned) + " misaligned")
		else:
			time.sleep(0.001)
	#print(" - arduinoInputThread returning")

# Draws graphs based on input from arduino
def drawGraphs(data,terminate):
	# Set up graphing
	fig,axes = subplots(1,2)
	fig.set_size_inches(20,10)
	#print(axes)
	for i in axes: i.set_xlim(auto=True)
	axes[0].set_ylim(-0.5,0.5)
	axes[1].set_ylim(0,70)
	
	t,roll,mv1 = [],[],[]
	fig.canvas.draw()
	fig.show()
	backgrounds = [ fig.canvas.copy_from_bbox(i.bbox) for i in axes ] # cache the background
	plt = [axes[0].plot(0,0,'-')[0],axes[1].plot(0,0,'-')[0]]
	
	# Draw graphs
	currentData = data[-1000:]
	loopStartTime = time.time()
	while(terminate[0] == False):
		
		loopStartTime = time.time()
		
		if len(data) < 2:
			time.sleep(0.01)
			continue
		
		#if(len(currentData) > 0):
		#	if(currentData[-1] == data[-1]): continue		# Check if we have some new data
		
		currentData = data[-1000:]						# Take snapshot of data *before* reading it into arrays
		t = [x[0] for x in currentData ]
		roll= [x[3] for x in currentData ]
		mv1=[x[7] for x in currentData ]
		
	
		plt[0].set_data(t,roll)							# update the xy data
		plt[1].set_data(t,mv1)							# update the xy data
		axes[0].set_xlim(t[0],t[-1])
		axes[1].set_xlim(t[0],t[-1])
		fig.canvas.restore_region(backgrounds[0])		# restore background
		fig.canvas.restore_region(backgrounds[1])		# restore background
		axes[0].draw_artist(plt[0])						# redraw just the points
		axes[1].draw_artist(plt[1])						# redraw just the points
		fig.canvas.blit(axes[0].bbox)					# fill in the axes rectangle
		fig.canvas.blit(axes[1].bbox)					# fill in the axes rectangle
		
		drawEndTime = time.time()
		
		# We only want to update graph at 30 FPS so sleep until this loop iteration has taken 1/30th of a second
		if(drawEndTime-loopStartTime < (1.0/30.0)):
			#print((1.0/30.0)-(drawEndTime-loopStartTime))
			time.sleep((1.0/30.0) - (drawEndTime-loopStartTime))
		
		#Calculate the frame rate
		loopEndTime = time.time()
		frequency = 1.0/(loopEndTime-loopStartTime)
		#print("FPS: " + str(frequency) + " ST: " + str(loopStartTime) + " ET: " + str(loopEndTime))
	close(fig)
	#print(" - drawGraphsThread returning")

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

while(True):
	#Open serial port
	newPort = serial.Serial(portList[portNum-1][0],baudrate=115200)
	print("\nOpened " + newPort.portstr)
	
	#Check Arduino is responding
	print("Checking if Arduino has initialised properly")
	time.sleep(3)
	
	b = [0]*newPort.inWaiting()
	newPort.readinto(b)
	if(''.join(b[-56:]) == 'Send any character to begin DMP programming and demo: \r\n'):
		print("Arduino and sensor successfully initialised!")
	else:
		print("Error: Arduino didn't initialise properly, trying again.")
		print("Closing " + newPort.portstr)
		newPort.close()
		continue
	
	print("\nInitialising DMP")
	newPort.flushInput()
	newPort.write('h')
	time.sleep(3)
	
	b = [0]*167
	newPort.readinto(b)
	if(''.join(b[-22:]) == 'FIFO Packet size: 48\r\n'):
		print("DMP successfully initialised!")
		break
	else:
		print("Error: DMP didn't initialise properly, trying again.")
		print("Closing " + newPort.portstr)
		newPort.close()
		continue

data = []
terminate = [[False],[False],[False]]
try:
	#newPort.write('on')
	# Start threads
	print("\nStarting serial port listener thread")
	arduinoInputThread = threading.Thread(target=arduinoInput,args=(newPort,data,terminate[0]))
	arduinoInputThread.daemon=True
	arduinoInputThread.start()
	print("Started!")
	
	print("\nStarting user input handling thread")
	consoleInputThread = threading.Thread(target=consoleInput,args=(newPort,terminate[1]))
	consoleInputThread.daemon=True
	consoleInputThread.start()
	print("Started!")
	
	print("\nStarting graph drawing thread")	
	drawThread = threading.Thread(target=drawGraphs,args=(data,terminate[2]))
	drawThread.daemon=True
	drawThread.start()
	print("Started!")
	
	while(True): time.sleep(10000)
except KeyboardInterrupt:
	print("\nCaught Ctrl-C :(. I thought you liked me...")
	
	print("\nTerminating threads:")
	terminate[0][0] = True
	arduinoInputThread.join()
	print(" - arduinoInputThread returned")
	terminate[1][0] = True
	consoleInputThread.join()
	print(" - consoleInputThread returned")
	terminate[2][0] = True
	drawThread.join()
	print(" - drawGraphsThread   returned")
	
	print("\nTurning off motors:")
	newPort.write('off')
	newPort.close()
	print("Motors should be off now :)")
	
	outputFile = open("speeddump",'w')
	for i in data:
		outputFile.write(str(i[0]) + "," + str(i[1]) + "\n")
	
	print("\nExiting...")

