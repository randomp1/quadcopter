import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.serial.*; 
import processing.opengl.*; 
import java.util.*; 
import java.util.concurrent.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class graph extends PApplet {

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
//	| char '>' | uint32_t time | float (32-bit) roll | float (32-bit) motor1 | float (32-bit) motor2 |
//






// ================================================================
// ===							CLASSES						 	===
// ================================================================

class pair
{
	double xVar;
	double yVar;

	//Constructors
	pair(double newX,double newY) { xVar = newX; yVar = newY; }
	
	//Setter and getter methods
	public void setVarX(double newX) { xVar = newX; }
	public void setVarY(double newY) { yVar = newY; }
	public void setPair(double newX,double newY) {xVar = newX; yVar = newY; }
	
	public double getVarX() { return xVar; }
	public double getVarY() { return yVar; }
}

class dataPacket
{
	//Data members
	long time;
	float [] ypr = new float[3];
	float [] pid = new float[3];
	float [] motorValues = new float[4];
	
	//Constructors
	dataPacket()
	{
		time=0;
		ypr[0]=0;
		ypr[1]=0;
		ypr[2]=0;
		pid[0] = 0;
		pid[1] = 0;
		pid[2] = 0;
		motorValues[0] = 0;
		motorValues[1] = 0;
		motorValues[2] = 0;
		motorValues[3] = 0;
	}
	
	dataPacket(long timeInit, float yaw, float pitch, float roll, float p, float i, float d, float MV1, float MV2, float MV3, float MV4)
	{
		time = timeInit;
		ypr[0] = yaw;
		ypr[1] = pitch;
		ypr[2] = roll;
		pid[0] = p;
		pid[1] = i;
		pid[2] = d;
		motorValues[0] = MV1;
		motorValues[1] = MV2;
		motorValues[2] = MV3;
		motorValues[3] = MV4;
	}
	
	//Setter methods
	public void setTime(long timeNew) { time = timeNew; }
	
	public void setYawPitchRoll(float yaw, float pitch, float roll)
	{
		ypr[0] = yaw;
		ypr[1] = pitch;
		ypr[2] = roll;
	}
	
	public void setYawPitchRoll(float [] yawPitchRoll)
	{
		if(yawPitchRoll.length < 3)
		{
			println("Error in setYawPitchRoll(float [] yawPitchRoll), array length less than three!");
			System.exit(0);
		}
		
		System.arraycopy(yawPitchRoll,0,ypr,0,3);
	}
	
	//Getter methods
	public long getTime() { return time; }
	public float getYaw() { return ypr[0]; }
	public float getPitch() { return ypr[1]; }
	public float getRoll() { return ypr[2]; }
	public void getYawPitchRoll(float [] yawPitchRoll)
	{
		if(yawPitchRoll.length < 3)
		{
			println("Error in getYawPitchRoll(float [] yawPitchRoll), array length less than three!");
			System.exit(0);
		}
		
		System.arraycopy(ypr,0,yawPitchRoll,0,3);
	}
	public float getYawPitchRoll(int valueToSelect)
	{
		if((valueToSelect >= 0) && (valueToSelect <= 2)) return ypr[valueToSelect];
		else
		{
			println("Error in getYawPitchRoll(int valueToSelect), value outside allowable range!");
			System.exit(0);
			return 0.0f;
		}
	}
	public void getMotorValues(float [] motorValues)
	{
		if(motorValues.length < 4)
		{
			println("Error in getMotorValues(float [] motorValues), array length less than 4");
			System.exit(0);
		}
	}
	public float getMotorValues(int valueToSelect)
	{
		if((valueToSelect >= 0) && (valueToSelect <= 3)) return motorValues[valueToSelect];
		else
		{
			println("Error in getMotorValues(int valueToSelect), value outside allowable range!");
			System.exit(0);
			return 0.0f;
		}
	}
}

class plot
{
	//Scale and position of graph relative to window (set by user)
	double ratioHeightToWindowHeight;
	double ratioWidthToWindowWidth;
	double ratioVPosToWindowHeight;
	double ratioHPosToWindowWidth;
	
	//Maximum and minimum axis values (automatically calculated or set by user)
	int autoScale;		//0 = manual, 1 = automatic
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
	
	//Constructor
	plot(int newNumDataPoints,double hScale,double vScale,double hPos,double vPos,int xNumLabels,int yNumLabels)
	{
		numDataPoints = newNumDataPoints;
		ratioHeightToWindowHeight = vScale;
		ratioWidthToWindowWidth = hScale;
		ratioVPosToWindowHeight = vPos;
		ratioHPosToWindowWidth = hPos;
		xAxisNumLabels = xNumLabels;
		yAxisNumLabels = yNumLabels;
		autoScale = 0;
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
		autoScale = 1;
		yScaleMax = yScaleMaxNew;
		yScaleMin = yScaleMinNew;
	}
	
	public void setDataToPlot(pair [] newDataToPlot)
	{
		dataToPlot = newDataToPlot;
	}
	
	public int getNumDataPoints() { return numDataPoints; }
	
	public void drawGraph()
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
			if(autoScale == 0) yScaleMin = dataToPlot[0].getVarY();
			if(autoScale == 0) yScaleMax = dataToPlot[0].getVarY();
	
			//Run through array to find highest and lowest x and y values
			for(int i=0; i<dataToPlot.length; i++)
			{
				//println("i: " + i);
				if(dataToPlot[i].getVarX() > xScaleMax) xScaleMax = dataToPlot[i].getVarX();
				else if(dataToPlot[i].getVarX() < xScaleMin) xScaleMin = dataToPlot[i].getVarX();
		
				if(autoScale == 0)
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
				xTextPos = xShift+scaledWidth/10.0f+((double)i/(xAxisNumLabels-1.0f)*(8.0f*scaledWidth/10.0f));
				yTextPos = yShift+scaledHeight-scaledHeight/20.0f;
				text((float)(xScaleMax-i*(xScaleMax-xScaleMin)/(xAxisNumLabels-1)),(int) xTextPos,(int) yTextPos);
			}
			
			for(int i=0; i < yAxisNumLabels; i++)
			{
				xTextPos = xShift+scaledWidth/20.0f;
				yTextPos = yShift+scaledHeight/10.0f+((double)i/(yAxisNumLabels-1.0f)*(8.0f*scaledHeight/10.0f));
				text((float)(yScaleMax-i*(yScaleMax-yScaleMin)/(yAxisNumLabels-1.0f)),(int) xTextPos,(int) yTextPos);
			}
		
			//Plot data:
			for(int i=0; i<(dataToPlot.length-1); i++)
			{
				//Define x and y coordinates of points between which to draw lines
				int x1 = (int) (xShift+(dataToPlot[i].getVarX()-xScaleMin)/(xScaleMax-xScaleMin)*(4.0f*scaledWidth/5.0f) + scaledWidth/10.0f);
				int y1 = (int) (yShift+(yScaleMax-dataToPlot[i].getVarY())/(yScaleMax-yScaleMin)*(4.0f*scaledHeight/5.0f) + scaledHeight/10.0f);
				int x2 = (int) (xShift+(dataToPlot[i+1].getVarX()-xScaleMin)/(xScaleMax-xScaleMin)*(4.0f*scaledWidth/5.0f) + scaledWidth/10.0f);
				int y2 = (int) (yShift+(yScaleMax-dataToPlot[i+1].getVarY())/(yScaleMax-yScaleMin)*(4.0f*scaledHeight/5.0f) + scaledHeight/10.0f);
			
				//Draw line
				line(x1,y1,x2,y2);
			}
		}
	}
}

Serial port;						 // The serial port
char[] packet = new char[47];	 	 // Packet
int serialCount = 0;				 // current packet byte position
int aligned = 0;
int interval = 0;

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

//Keyboard input stuff
String keyBuffer = "";

//Height and width of window
int height = 700;
int width = 1000;

public void setup()
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
	
	rollPlot = new plot(2000,1.0f,0.5f,0,0.5f,11,11,-0.5f,0.5f);
	rollPlot.setDataToPlot(rollCoordList.toArray(new pair[rollCoordList.size()]));
	
	//Set up motor 1 plot
	MV1CoordList = new LinkedBlockingDeque<pair>();
	
	MV1Plot = new plot(1000,0.5f,0.5f,0,0,11,11,40,50);
	MV1Plot.setDataToPlot(MV1CoordList.toArray(new pair[MV1CoordList.size()]));
	
	//Set up motor 2 plot
	MV2CoordList = new LinkedBlockingDeque<pair>();
	
	MV2Plot = new plot(1000,0.5f,0.5f,0.5f,0,11,11,40,50);
	MV2Plot.setDataToPlot(MV2CoordList.toArray(new pair[MV2CoordList.size()]));

	// display serial port list for debugging/clarity
	println(Serial.list());
	
	if(Serial.list().length < 1)
	{
		println("No serial ports detected!\nTerminating now...");
		System.exit(0);
	}

	// get the first available port (use EITHER this OR the specific port code below)
	String portName = Serial.list()[0];
	
	// get a specific serial port (use EITHER this OR the first-available code above)
	//String portName = "COM4";
	
	// open the serial port
	port = new Serial(this, portName, 115200);
	
	// send single character to trigger DMP init/start
	// (expected by MPU6050_DMP6 example Arduino sketch)
	delay(1000);
	port.write('r');
	delay(2000);
	port.write("on");
}

public void draw()
{
	if(millis() - interval > 1000)
	{
		port.write('r');
		interval = millis();
	}
	
	// black background
	background(255,255,255);
	
	height = frame.getHeight()-30;
	width = frame.getWidth()-10;
	
	rollPlot.drawGraph();
	MV1Plot.drawGraph();
	MV2Plot.drawGraph();
}

public void serialEvent(Serial port)
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
					
					//Add new data packet to end of deque and remove last element if there are more than numDataPoints data points stored
					newDataPacket = new dataPacket(tempInt,Float.intBitsToFloat(tempYaw),Float.intBitsToFloat(tempPitch),Float.intBitsToFloat(tempRoll),0,0,0,Float.intBitsToFloat(tempMV1),Float.intBitsToFloat(tempMV2),0,0);
					
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
			}
		}
	}
}

public void keyPressed()
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
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "graph" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
