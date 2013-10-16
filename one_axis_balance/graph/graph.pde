// I2C device class (I2Cdev) demonstration Processing sketch for MPU6050 DMP output
// 6/20/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//	 2012-06-20 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

//Packet format:
//
//	| char '>' | uint32_t time | float (32-bit) yaw | float (32-bit) pitch | float (32-bit) roll | float (32-bit) motor1 | float (32-bit) motor2 |
//

import processing.serial.*;
import processing.opengl.*;
import java.util.*;
import java.util.concurrent.*;

// ================================================================
// ===							CLASSES						 	===
// ================================================================

class plot
{
	//Scale and position of graph relative to window (set by user)
	double ratioHeightToWindowHeight;
	double ratioWidthToWindowWidth;
	double ratioVPosToWindowHeight;
	double ratioHPosToWindowWidth;
	
	//Maximum and minimum axis values (automatically calculated or set by user)
	boolean autoScale;		//false = manual, true = automatic
	double xScaleMax;
	double xScaleMin;
	double yScaleMax;
	double yScaleMin;
	
	//Number of axis labels (set by user)
	int xAxisNumLabels;
	int yAxisNumLabels;
	
	//Data to plot
	int numDataPoints;
	pair [] dataToPlot;
	
	//Constructors
	plot(int newNumDataPoints,double hScale,double vScale,double hPos,double vPos,int xNumLabels,int yNumLabels)
	{
		numDataPoints = newNumDataPoints;
		ratioHeightToWindowHeight = vScale;
		ratioWidthToWindowWidth = hScale;
		ratioVPosToWindowHeight = vPos;
		ratioHPosToWindowWidth = hPos;
		xAxisNumLabels = xNumLabels;
		yAxisNumLabels = yNumLabels;
		autoScale = true;
	}
	
	plot(int newNumDataPoints,double hScale,double vScale,double hPos,double vPos,int xNumLabels,int yNumLabels,double yScaleMinNew,double yScaleMaxNew)
	{
		numDataPoints = newNumDataPoints;
		ratioHeightToWindowHeight = vScale;
		ratioWidthToWindowWidth = hScale;
		ratioVPosToWindowHeight = vPos;
		ratioHPosToWindowWidth = hPos;
		xAxisNumLabels = xNumLabels;
		yAxisNumLabels = yNumLabels;
		autoScale = false;
		yScaleMax = yScaleMaxNew;
		yScaleMin = yScaleMinNew;
	}
	
	void setDataToPlot(pair [] newDataToPlot)
	{
		dataToPlot = newDataToPlot;
	}
	
	int getNumDataPoints() { return numDataPoints; }
	
	void drawGraph()
	{
		//Scale and translate lengths as required
		double scaledWidth = width*ratioWidthToWindowWidth;
		double scaledHeight = height*ratioHeightToWindowHeight;
		double xShift = width*ratioHPosToWindowWidth;
		double yShift = height*ratioVPosToWindowHeight;
		
		//Draw axes
		line((float)(xShift+scaledWidth/10),(float)(yShift+scaledHeight/10),(float)(xShift+(scaledWidth/10)),(float)(yShift+scaledHeight-(scaledHeight/10)));
		line((float)(xShift+scaledWidth/10),(float)(yShift+scaledHeight-(scaledHeight/10)),(float)(xShift+scaledWidth-(scaledWidth/10)),(float)(yShift+scaledHeight-(scaledHeight/10)));
	
		//Only put data on graph if there are two points between which a line can be drawn
		if(dataToPlot.length > 1)
		{
			//Check if min and max values of x and y have changed
			xScaleMin = dataToPlot[0].getVarX();
			xScaleMax = dataToPlot[0].getVarX();
			if(autoScale == true) yScaleMin = dataToPlot[0].getVarY();
			if(autoScale == true) yScaleMax = dataToPlot[0].getVarY();
	
			//Run through array to find highest and lowest x and y values
			for(int i=0; i<dataToPlot.length; i++)
			{
				if(dataToPlot[i].getVarX() > xScaleMax) xScaleMax = dataToPlot[i].getVarX();
				else if(dataToPlot[i].getVarX() < xScaleMin) xScaleMin = dataToPlot[i].getVarX();
		
				if(autoScale == true)
				{
					if(dataToPlot[i].getVarY() > yScaleMax) yScaleMax = dataToPlot[i].getVarY();
					else if(dataToPlot[i].getVarY() < yScaleMin) yScaleMin = dataToPlot[i].getVarY();
				}
			}
			
			//label axes
			fill(0,0,0);
			textSize(11);
			
			double xTextPos;
			double yTextPos;
			for(int i=0; i < xAxisNumLabels; i++)
			{
				xTextPos = xShift+scaledWidth/10.0+((double)i/(xAxisNumLabels-1.0)*(8.0*scaledWidth/10.0));
				yTextPos = yShift+scaledHeight-scaledHeight/20.0;
				text((float)(xScaleMin+i*(xScaleMax-xScaleMin)/(xAxisNumLabels-1)),(int) xTextPos,(int) yTextPos);
			}
			
			for(int i=0; i < yAxisNumLabels; i++)
			{
				xTextPos = xShift+scaledWidth/20.0;
				yTextPos = yShift+scaledHeight/10.0+((double)i/(yAxisNumLabels-1.0)*(8.0*scaledHeight/10.0));
				text((float)(yScaleMax-i*(yScaleMax-yScaleMin)/(yAxisNumLabels-1.0)),(int) xTextPos,(int) yTextPos);
			}
		
			//Plot data:
			for(int i=0; i<(dataToPlot.length-1); i++)
			{
				//Check if graph scale allows data point to be plotted (should only not be the case if manual scale is used)
				if( ((dataToPlot[i].getVarX() > xScaleMax) || (dataToPlot[i].getVarX() < xScaleMin) || (dataToPlot[i].getVarY() > yScaleMax) || (dataToPlot[i].getVarY() < yScaleMin))
				 || ((dataToPlot[i+1].getVarX() > xScaleMax) || (dataToPlot[i+1].getVarX() < xScaleMin) || (dataToPlot[i+1].getVarY() > yScaleMax) || (dataToPlot[i+1].getVarY() < yScaleMin)) )
				{
					if(autoScale == true) println("MinSX:" + xScaleMin + " MaxSX:" + xScaleMax + " MinSY:" + yScaleMin + " MaxSY:" + yScaleMax + " (" + dataToPlot[i].getVarX() + "," + dataToPlot[i].getVarY() + ")");
					continue;
				}
				
				//Define x and y coordinates of points between which to draw lines
				int x1 = (int) (xShift+(dataToPlot[i].getVarX()-xScaleMin)/(xScaleMax-xScaleMin)*(4.0*scaledWidth/5.0) + scaledWidth/10.0);
				int y1 = (int) (yShift+(yScaleMax-dataToPlot[i].getVarY())/(yScaleMax-yScaleMin)*(4.0*scaledHeight/5.0) + scaledHeight/10.0);
				int x2 = (int) (xShift+(dataToPlot[i+1].getVarX()-xScaleMin)/(xScaleMax-xScaleMin)*(4.0*scaledWidth/5.0) + scaledWidth/10.0);
				int y2 = (int) (yShift+(yScaleMax-dataToPlot[i+1].getVarY())/(yScaleMax-yScaleMin)*(4.0*scaledHeight/5.0) + scaledHeight/10.0);
			
				//Draw line
				line(x1,y1,x2,y2);
			}
		}
	}
}

//Input stream parameters
boolean useSerial;
Serial port;						 // The serial port
char[] packet = new char[47];	 	 // Packet
int serialCount = 0;				 // current packet byte position
int aligned = 0;
int interval = 0;
String sampleData [];

//Graph parameters
plot rollPlot;
plot MV1Plot;
plot MV2Plot;
int numAxisLabels_y = 11;
int numAxisLabels_x = 11;
float max_y = 10;
float min_y = 0;
float max_x = 10;
float min_x = 0;

//Data
dataPacket newDataPacket;
LinkedBlockingDeque<pair> rollCoordList;
LinkedBlockingDeque<pair> MV1CoordList;
LinkedBlockingDeque<pair> MV2CoordList;
LinkedBlockingDeque<pair> PCoordList;
LinkedBlockingDeque<pair> ICoordList;
LinkedBlockingDeque<pair> DCoordList;
long previousTime = 0;
long latestTime = 0;

//Keyboard input stuff
String keyBuffer = "";

//Height and width of window
int height = 700;
int width = 1000;

void setup()
{
	// 300px square viewport using OpenGL rendering
	size(width, height, P2D);
	if(frame != null) frame.setResizable(true);
	
	println("Frame width: " + frame.getWidth());
	println("Frame height: " + frame.getHeight());
	
	// setup lights and antialiasing
	smooth();
	
	//Initialise packet array
	for(int i=0; i<47; i++) packet[i] = 0;
	
	//Set up roll plot
	rollCoordList = new LinkedBlockingDeque<pair>();
	
	rollPlot = new plot(2000,1.0,0.5,0,0.5,11,11,-0.5,0.5);
	rollPlot.setDataToPlot(rollCoordList.toArray(new pair[rollCoordList.size()]));
	
	//Set up PID deques
	PCoordList = new LinkedBlockingDeque<pair>();
	ICoordList = new LinkedBlockingDeque<pair>();
	DCoordList = new LinkedBlockingDeque<pair>();
	
	//Set up motor 1 plot
	MV1CoordList = new LinkedBlockingDeque<pair>();
	
	MV1Plot = new plot(1000,0.5,0.5,0,0,11,11,40,50);
	MV1Plot.setDataToPlot(MV1CoordList.toArray(new pair[MV1CoordList.size()]));
	
	//Set up motor 2 plot
	MV2CoordList = new LinkedBlockingDeque<pair>();
	
	MV2Plot = new plot(1000,0.5,0.5,0.5,0,11,11,40,50);
	MV2Plot.setDataToPlot(MV2CoordList.toArray(new pair[MV2CoordList.size()]));

	// display serial port list for debugging/clarity
	println(Serial.list());
	
	if(Serial.list().length < 1)
	{
		println("No serial ports detected!\nUsing sample csv file.");
		
		//load strings from sample file and process them
		sampleData = loadStrings("./sample_data.csv");
		
		for(int i=0; i<sampleData.length; i++)
		{
			String tempString [] = sampleData[i].split(",");
			if(tempString.length < 11)
			{
				println("Error! tempString too short!");
				System.exit(0);
			}
			
			//Create new dataPacket to store results in a nice format
			newDataPacket = new dataPacket((new Float(tempString[0])).longValue(),
											new Float(tempString[1]),
											new Float(tempString[2]),
											new Float(tempString[3]),
											new Float(tempString[4]),
											new Float(tempString[5]),
											new Float(tempString[6]),
											new Float(tempString[7]),
											new Float(tempString[8]),
											new Float(tempString[9]),
											new Float(tempString[10]));
			
			//Using new data update graph coordinate lists
			rollCoordList.push(new pair((double)newDataPacket.getTime(),(double)newDataPacket.getRoll()));
			if(rollCoordList.size() > rollPlot.getNumDataPoints()) rollCoordList.removeLast();
			
			rollPlot.setDataToPlot(rollCoordList.toArray(new pair[rollCoordList.size()]));
			
			MV1CoordList.push(new pair((double)newDataPacket.getTime(),(double)newDataPacket.getMotorValues(0)));
			if(MV1CoordList.size() > MV1Plot.getNumDataPoints()) MV1CoordList.removeLast();
			
			MV1Plot.setDataToPlot(MV1CoordList.toArray(new pair[MV1CoordList.size()]));
			
			MV2CoordList.push(new pair((double)newDataPacket.getTime(),(double)newDataPacket.getMotorValues(1)));
			if(MV2CoordList.size() > MV2Plot.getNumDataPoints()) MV2CoordList.removeLast();
			
			MV2Plot.setDataToPlot(MV2CoordList.toArray(new pair[MV2CoordList.size()]));
		}
		
		//Indicate that serial input is not being used		
		useSerial=false;
	}
	else
	{
		// get the first available port (use EITHER this OR the specific port code below)
		String portName = Serial.list()[0];
	
		// get a specific serial port (use EITHER this OR the first-available code above)
		//String portName = "COM4";
	
		// open the serial port
		port = new Serial(this, portName, 115200);
		
		//Indicate that serial input is being used
		useSerial=true;
	
		// send single character to trigger DMP init/start
		// (expected by MPU6050_DMP6 example Arduino sketch)
		delay(1000);
		port.write('r');
		delay(2000);
		port.write("on");
	}
}

void draw()
{
	if((millis() - interval > 1000) && (useSerial == true))
	{
		port.write('r');
		interval = millis();
	}
	
	// White background
	background(255,255,255);

	text("Framerate: " + frameRate,10,15);
	if(previousTime != latestTime)
	{
		text("Data frequency: " + (1000.0/(latestTime-previousTime)),10,25);
		text("P: " + (PCoordList.peekLast()).getVarY(),200,15);
		text("I: " + (ICoordList.peekLast()).getVarY(),400,15);
		text("D: " + (DCoordList.peekLast()).getVarY(),600,15);
	}
	
	height = frame.getHeight()-30;
	width = frame.getWidth()-10;

	rollPlot.drawGraph();
	MV1Plot.drawGraph();
	MV2Plot.drawGraph();
}

void serialEvent(Serial port)
{
	interval = millis();
	while (port.available() > 0)
	{
		int ch = port.read();
		//print((char)ch);
		if (aligned < 3)
		{
			// make sure we are properly aligned on a 47-byte packet
			if(ch == '$')
			{
				aligned++;
				serialCount = 0;
			}
			else if (serialCount == 45) { if (ch == '\r') aligned++; else aligned = 0; }
			else if (serialCount == 46) { if (ch == '\n') aligned++; else aligned = 0; }
			//println((char)ch + " " + aligned + " " + serialCount);
			serialCount++;
			if (serialCount == 47) serialCount = 0;
		}
		else
		{
			if (serialCount > 0 || ch == '$')
			{
				//println("SerialCount: " + serialCount);
				packet[serialCount++] = (char)ch; //the expression is SerialCount, SerialCount itself is incremented
				if (serialCount == 47)
				{
					serialCount = 0; // restart packet byte position
					//println("Here!");
					// get time from data packet
					long tempInt = 0;
					tempInt = (tempInt | packet[1]) << 8;
					tempInt = (tempInt | packet[2]) << 8;
					tempInt = (tempInt | packet[3]) << 8;
					tempInt = (tempInt | packet[4]);
					
					//Get yaw from data packet
					int tempYaw = 0;
					tempYaw = (tempYaw | packet[5]) << 8;
					tempYaw = (tempYaw | packet[6]) << 8;
					tempYaw = (tempYaw | packet[7]) << 8;
					tempYaw = (tempYaw | packet[8]);
					
					//Get pitch from data packet
					int tempPitch = 0;
					tempPitch = (tempPitch | packet[9]) << 8;
					tempPitch = (tempPitch | packet[10]) << 8;
					tempPitch = (tempPitch | packet[11]) << 8;
					tempPitch = (tempPitch | packet[12]);
					
					//Get roll from data packet
					int tempRoll = 0;
					tempRoll = (tempRoll | packet[13]) << 8;
					tempRoll = (tempRoll | packet[14]) << 8;
					tempRoll = (tempRoll | packet[15]) << 8;
					tempRoll = (tempRoll | packet[16]);
					
					//Get P coefficient from packet
					int tempP = 0;
					tempP = (tempP | packet[17]) << 8;
					tempP = (tempP | packet[18]) << 8;
					tempP = (tempP | packet[19]) << 8;
					tempP = (tempP | packet[20]);
					
					//Get I coefficient from packet
					int tempI = 0;
					tempI = (tempI | packet[21]) << 8;
					tempI = (tempI | packet[22]) << 8;
					tempI = (tempI | packet[23]) << 8;
					tempI = (tempI | packet[24]);
					
					//Get D coefficient from packet
					int tempD = 0;
					tempD = (tempD | packet[25]) << 8;
					tempD = (tempD | packet[26]) << 8;
					tempD = (tempD | packet[27]) << 8;
					tempD = (tempD | packet[28]);
					
					//Get MV1 from data packet
					int tempMV1 = 0;
					tempMV1 = (tempMV1 | packet[29]) << 8;
					tempMV1 = (tempMV1 | packet[30]) << 8;
					tempMV1 = (tempMV1 | packet[31]) << 8;
					tempMV1 = (tempMV1 | packet[32]);
					
					//Get MV2 from data packet
					int tempMV2 = 0;
					tempMV2 = (tempMV2 | packet[33]) << 8;
					tempMV2 = (tempMV2 | packet[34]) << 8;
					tempMV2 = (tempMV2 | packet[35]) << 8;
					tempMV2 = (tempMV2 | packet[36]);
					
					previousTime = latestTime;
					latestTime = tempInt;
					
					//Create new dataPacket to store results in a nice format
					newDataPacket = new dataPacket(tempInt,
												   Float.intBitsToFloat(tempYaw),
												   Float.intBitsToFloat(tempPitch),
												   Float.intBitsToFloat(tempRoll),
												   Float.intBitsToFloat(tempP),
												   Float.intBitsToFloat(tempI),
												   Float.intBitsToFloat(tempD),
												   Float.intBitsToFloat(tempMV1),
												   Float.intBitsToFloat(tempMV2),
												   0,
												   0);
					
					//Using new data update graph coordinate lists
					rollCoordList.push(new pair((double)newDataPacket.getTime(),(double)newDataPacket.getRoll()));
					if(rollCoordList.size() > rollPlot.getNumDataPoints()) rollCoordList.removeLast();
					
					rollPlot.setDataToPlot(rollCoordList.toArray(new pair[rollCoordList.size()]));
					
					PCoordList.push(new pair((double)newDataPacket.getTime(),(double)newDataPacket.getP()));
					if(PCoordList.size() > 1) PCoordList.removeLast();
					
					ICoordList.push(new pair((double)newDataPacket.getTime(),(double)newDataPacket.getI()));
					if(ICoordList.size() > 1) ICoordList.removeLast();
					
					DCoordList.push(new pair((double)newDataPacket.getTime(),(double)newDataPacket.getD()));
					if(DCoordList.size() > 1) DCoordList.removeLast();
					
					MV1CoordList.push(new pair((double)newDataPacket.getTime(),(double)newDataPacket.getMotorValues(0)));
					if(MV1CoordList.size() > MV1Plot.getNumDataPoints()) MV1CoordList.removeLast();
					
					MV1Plot.setDataToPlot(MV1CoordList.toArray(new pair[MV1CoordList.size()]));
					
					MV2CoordList.push(new pair((double)newDataPacket.getTime(),(double)newDataPacket.getMotorValues(1)));
					if(MV2CoordList.size() > MV2Plot.getNumDataPoints()) MV2CoordList.removeLast();
					
					MV2Plot.setDataToPlot(MV2CoordList.toArray(new pair[MV2CoordList.size()]));
				}
			}
		}
	}
}

void keyPressed()
{
	if((key == '\n') || (key == '\r'))
	{
		println("String sent: " + keyBuffer);
		port.write(keyBuffer);
		keyBuffer = "";
	}
	else
	{
		keyBuffer += key;
	}	
}
