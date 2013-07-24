import serial

newPort = serial.Serial('/dev/ttyACM0')

print("Opened " + newPort.portstr)

print(newPort.readline())

newPort.close()
#serial.tools.list_ports.comports()
