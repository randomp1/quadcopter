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

# Reads input from the arduino
def arduinoInput(serialPort,data,terminate):
	previous_time = 0
	# Main loop reading and handling data
	alignment(serialPort)
	while(terminate[0] == False):#aligned < 1000):
		b = []
		b.append(serialPort.read())
		while(b[-1] != '\n'):
			b.append(serialPort.read())
			#print(b)
		
		if(b[0] == 'M'):
			print("Speed up at: " + "".join(b[1:-1]))
			continue
		current_time = int((''.join(b[:-1])))
		print(current_time)
		data.append([current_time,current_time - previous_time])
		previous_time = current_time
		
# Draws graphs based on input from arduino
def drawGraphs(data,terminate):
	# Set up graphing
	fig,axes = subplots(1,1)
	fig.set_size_inches(15,10,forward=True)
	#print(axes)
	axes.set_xlim(0,100000)
	axes.grid('on')
	axes.set_ylim(0,100)
	
	fig.canvas.draw()
	fig.show()
	backgrounds = fig.canvas.copy_from_bbox(axes.bbox) # cache the background
	plt = axes.plot(0,0,'-')[0]
	
	# Draw graphs
	currentData = data[-200:]
	loopStartTime = time.time()
	while(terminate[0] == False):
		
		loopStartTime = time.time()
		
		if len(data) < 2:
			time.sleep(0.01)
			continue
		
		#if(len(currentData) > 0):
		#	if(currentData[-1] == data[-1]): continue		# Check if we have some new data
		
		currentData = data[-200:]						# Take snapshot of data *before* reading it into arrays
		xValues = [ i[0] for i in currentData ]
		yValues = [ 0.5/(i[1]/1000.0) for i in currentData ]
	
		plt.set_data(xValues,yValues)							# update the xy data
		axes.set_xlim(xValues[0],xValues[-1])
		fig.canvas.restore_region(backgrounds)		# restore background
		axes.draw_artist(plt)						# redraw just the points
		#axes.draw_artist(axes)
		fig.canvas.blit(axes.bbox)					# fill in the axes rectangle
		
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

#Open serial port
newPort = serial.Serial(portList[portNum-1][0],baudrate=115200)
print("\nOpened " + newPort.portstr)
	


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
	terminate[2][0] = True
	drawThread.join()
	print(" - drawGraphsThread   returned")
	
	outputFile = open("speeddump",'w')
	for i in data:
		outputFile.write(str(i[0]) + "," + str(i[1]) + "\n")
	
	print("\nExiting...")

