// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//	 2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//	 2012-06-20 - improved FIFO overflow handling and simplified read process
//	 2012-06-19 - completely rearranged DMP initialization code and simplification
//	 2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//	 2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//	 2012-06-05 - add gravity-compensated initial reference frame acceleration output
//				- add 3D math helper file to DMP6 example sketch
//				- add Euler output and Yaw/Pitch/Roll output formats
//	 2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//	 2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//	 2012-05-30 - basic DMP initialization working

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

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include <Servo.h>

#include "MPU6050_9Axis_MotionApps41.h"
//#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

template<class T>
class fifo
{
	T* fifoArray;
	T* front; //Points to element at front of queue
	T* back; //Points to element at back of queue
	int numItems;
	int maxItems;
	
	public:
	fifo(int maxNumItems) //Constructor allocates a block of memory to be used for the duration of the object's life, no fancy stuff here!
	{
		if(maxNumItems <= 0) return;
		maxItems = maxNumItems;
		fifoArray = (T*) malloc((size_t) sizeof(T)*maxItems);
		numItems = 0;
		front = fifoArray;
		back = front;
	}
	
	fifo()
	{
		fifoArray = NULL;
		front = NULL;
		back = NULL;
		maxItems = 0;
		numItems = 0;
	}
	
	~fifo() //Deallocate the memory we allocated
	{
		free(fifoArray);
	}
	
	void set_size(unsigned int maxNumItems)
	{
		if(maxNumItems == 0) return;
		maxItems = maxNumItems;
		free(fifoArray);
		fifoArray = (T*) malloc((size_t) sizeof(T)*maxItems);
		numItems = 0;
		front = fifoArray;
		back = front;
	}
	
	void push_back(const T& item)
	{
		//Decide if and where to place item
		if(numItems == maxItems) return;
		else if(numItems != 0)
		{
			if(back != fifoArray) back--;
			else back = fifoArray+maxItems-1;
		}
		
		*back = item;
		numItems++;
	}
	
	void pop_back()
	{
		//Decide if we can remove an item
		if(numItems == 0) return;
		else if(numItems > 1)
		{
			if(back != fifoArray+maxItems-1) back++;
			else back = fifoArray;
		}
		
		numItems--;
	}
	
	void push_front(const T& item)
	{
		//Decide if and where to place item
		if(numItems == maxItems) return;
		else if(numItems != 0)
		{
			if(front != (fifoArray+maxItems-1))	front++;
			else front = fifoArray;
		}
		
		*front = item;
		numItems++;
	}
	
	void pop_front()
	{
		//Decide if we can remove an item
		if(numItems == 0) return;
		else if(numItems > 1)
		{
			if(front != fifoArray) front--;
			else front = fifoArray+maxItems-1;
		}
		
		numItems--;
	}
	
	T& operator[](int element)
	{
		if((element < numItems) && (element >= 0))
		{
			if(element + (back-fifoArray) >= maxItems)
			{
				element -= (maxItems-(back-fifoArray));
			}
			else element += back-fifoArray;
		}
		else return fifoArray[element+maxItems]; //TODO: find a better way of doing this!
		
		return fifoArray[element];
	}
	
	size_t size() { return numItems; }
	size_t max_size() { return maxItems; }
};

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

/* =========================================================================
	 NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
	 depends on the MPU-6050's INT pin being connected to the Arduino's
	 external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
	 digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
	 NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
	 when using Serial.write(buf, len). The Teapot output uses this method.
	 The solution requires a modification to the Arduino USBAPI.h file, which
	 is fortunately simple, but annoying. This will be fixed in the next IDE
	 release. For more info, see these links:

	 http://arduino.cc/forum/index.php/topic,109987.0.html
	 http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;	// set true if DMP init was successful
uint8_t mpuIntStatus;	// holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
uint8_t fifoBuffer[64];	// FIFO storage buffer

// orientation/motion vars
Quaternion q;			// [w, x, y, z]			quaternion container
VectorInt16 aa;			// [x, y, z]			accel sensor measurements
VectorInt16 aaReal;		// [x, y, z]			gravity-free accel sensor measurements
VectorInt16 aaWorld;	// [x, y, z]			world-frame accel sensor measurements
VectorFloat gravity;	// [x, y, z]			gravity vector
float euler[3];			// [psi, theta, phi]	Euler angle container
float ypr[3];			// [yaw, pitch, roll]	yaw/pitch/roll container and gravity vector

//PID loop variables
const float desired_angles[2]={0.0,0.0};	//desired angles when balanced
float previous_error[2]={0.0,0.0};			//initialise previous error
float integral[2] = {0.0,0.0};				//initialise integral
int previous_time = 0;
float pidConstants[3] = {0.05,1e-6,700};
float pidValues[3] = {0,0,0};
float defaultSpeed = 50;

const int numDataPoints = 3;
fifo<uint32_t> previousTimes;
fifo<float>* previousAngles;

//Output packet
//Format:				   { '$', time,    yaw,     pitch,   roll,    p-term,  i-term,  d-term,  motor1,  motor2,  motor3,  motor4,  '\r', '\n' }
uint8_t outputPacket[47] = { '$', 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, '\r', '\n' };

//Set up serial variables
String incoming_string;

//PSU control data
const int psuPin = 6;	//PSU pin
int psuState = 0;		//PSU on or off

//Motor control data
const int ESC_min_pulse = 1000;					//Minimum ESC pulse duration
const int ESC_max_pulse = 2000;					//Maximum ESC pulse duration
const int motor_pin[4] = {10,11,9,8};			//Pin assignment
Servo motor[4];									//Servo objects for motor
const int min_signal = 40;						//Minimum signal required to turn on motor
const int max_signal = 60;						//Maximum allowed signal (for safety!)
float motor_value_float[4] = {0.0,0.0,0.0,0.0};	//Array of motor values

//Function to raise an integer to a power (Be careful! Has serious limitations)
int powint(int number,int exponent);

//Function to parse input strings
int parse_input(String input_string);

//Function to implement PID loop
void updateMotors(const float *pidConstants,float *motor_value_float,const float* desired_angles,fifo<uint32_t>& timeArray,fifo<float>* angleArray,float *integral);

//Fits parabola of form y=ax^2+bx+c to data
void fitParabola(fifo<uint32_t> &xVals,fifo<float> &yVals,float &a,float &b,float &c);

//Function to store a float (32-bit floating point) in a four byte array
void floatToByteArray(float &floatToConvert, uint8_t *byteArray);

// ================================================================
// ===			   INTERRUPT DETECTION ROUTINE					===
// ================================================================

volatile bool mpuInterrupt = false;	 // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; }

// ================================================================
// ===					  INITIAL SETUP							===
// ================================================================

void setup()
{
	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();

	// initialize serial communication
	// (115200 chosen because it is required for Teapot Demo output, but it's
	// really up to you depending on your project)
	Serial.begin(115200);
	//while (!Serial); // wait for Leonardo enumeration, others continue immediately
	
	// NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
	// Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
	// the baud timing being too misaligned with processor ticks. You must use
	// 38400 or slower in these cases, or use some kind of external separate
	// crystal solution for the UART timer.
	
	// configure LED for output
	pinMode(LED_PIN, OUTPUT);
	
	//Set up FIFO buffers for data
	previousTimes.set_size(numDataPoints);
	previousAngles = (fifo<float>*) malloc(3*sizeof(fifo<float>));
	for(int i=0; i<3; i++)
	{
		previousAngles[i] = fifo<float>();	
		previousAngles[i].set_size(numDataPoints);
	}
	
	//Set motor pins as output and initialise the servo communications. Set the motors to be off!
	for(int i=0; i < 4; i++) pinMode(motor_pin[i],OUTPUT);
	for(int i=0; i < 4; i++) motor[i].attach(motor_pin[i],1000,2000); //this should really be 1100, 1800, but the ESC doesn't seem to accept it
	for(int i=0; i < 4; i++) motor[i].write(0);
	
	//Set PSU power signal pin (for turning it on)
	pinMode(psuPin,OUTPUT);
	digitalWrite(psuPin,HIGH);
	
	pinMode(2,INPUT);
	
	Serial.println("Motor controller ready :)");

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// wait for ready
	Serial.println(F("\nSend any character to begin DMP programming and demo: "));
	while (Serial.available() && Serial.read());	// empty buffer
	while (!Serial.available());					// wait for data
	while (Serial.available() && Serial.read());	// empty buffer again

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();
	
	// make sure it worked (returns 0 if so)
	if (devStatus == 0)
	{
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
		Serial.print(F("FIFO Packet size: "));
		Serial.println(packetSize);
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
}

// ================================================================
// ===					MAIN PROGRAM LOOP					 	===
// ================================================================

uint32_t time2, time1 = 0;

void loop()
{
	// if programming failed, don't try to do anything
	if (!dmpReady) return;
	
	time2 = micros()-time1;
	//Serial.println(time2);
	
	
	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize)
	{
		//Check if stuff is available on the serial input buffer
		if(Serial.available() > 0)
		{
			//Reset storage string
			incoming_string = 0;
		
			//Read in data and wait to see if more data is incoming for reading.
			while(Serial.available() != 0)
			{
				while(Serial.available() > 0) { incoming_string += String((char) Serial.read()); }
				delayMicroseconds(1000);
			}
		
			//Check if something's newly available on the serial port (indicates error)
			if(Serial.available() > 0) Serial.println("Error: timing wrong");
		
			//Parse string and indicate if recognised or not
			if(parse_input(incoming_string) == 1)
			{
				Serial.print("Error: incoming string (\"");
				Serial.print(incoming_string);
				Serial.print("\") unrecognised\n");
			}
			else
			{
				Serial.print("Incoming string (\"");
				Serial.print(incoming_string);
				Serial.print("\") recognised!\n"); 
			}
		}
	}
	
	time1 = micros();

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024)
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		fifoCount = 0;
		Serial.println(F("FIFO overflow!"));
		
	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & 0x02)
	{
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		//Record time of sensor reading
		uint32_t timer1 = millis();
		
		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		mpu.dmpGetQuaternion(&q, fifoBuffer);			//Takes ~70us		
//		mpu.dmpGetEuler(euler, &q);
//		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);				//Takes ~140us
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);		//Takes ~750us
//		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//		mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
		
		//Add our new data to the queues
		while(previousTimes.size() >= previousTimes.max_size()) previousTimes.pop_back();
		previousTimes.push_front(micros()/100); //TODO: Sort out overflow issue!
		
		for(int i=0; i<3; i++)
		{
			while(previousAngles[i].size() >= previousAngles[i].max_size()) previousAngles[i].pop_back();
			previousAngles[i].push_front(ypr[i]);
		}
		
		//Set motor throttle settings in arduino memory (ESC not told at this stage)
		if(psuState == 0) for(int i=0; i<4; i++) motor_value_float[i] = 0.0; //Zero if PSU is off
		else updateMotors(pidConstants,motor_value_float,desired_angles,previousTimes,previousAngles,integral); //PID loop otherwise
		
		//Send signal to ESC to change motor speed
		float pulseLength[4];
		for(int i=0; i < 2; i++)
		{
			pulseLength[i] = ESC_min_pulse + (motor_value_float[i]/180.0)*(ESC_max_pulse-ESC_min_pulse);
			if(pulseLength[i] > 1400) pulseLength[i] = 1000;
		}
		for(int i=0; i < 2; i++) if(motor_value_float[i] < max_signal) motor[i].writeMicroseconds(ESC_min_pulse + motor_value_float[i]*(ESC_max_pulse-ESC_min_pulse)/180.0);

		//Prepare output packet
		outputPacket[1] = (0xFF000000 & timer1) >> 24;			//Time
		outputPacket[2] = (0x00FF0000 & timer1) >> 16;
		outputPacket[3] = (0x0000FF00 & timer1) >> 8;
		outputPacket[4] = (0x000000FF & timer1);
		
		for(int i=0; i<3; i++) floatToByteArray(ypr[i],(outputPacket+5+(4*i)));	//Yaw, pitch and roll
		for(int i=0; i<3; i++) floatToByteArray(pidConstants[i],(outputPacket+17+(4*i))); //P,I and D
		for(int i=0; i<4; i++) floatToByteArray(motor_value_float[i],(outputPacket+29+(4*i))); //Motor 1-4
		
		//Send packet over serial port
		
		Serial.write(outputPacket,47); //Takes ~420us
		
/*		
		//Write ESC values to serial port
		Serial.print("T.E.:\t");
		Serial.print(previous_time);
		Serial.print("M1:\t");
		Serial.print(motor_value_float[0]);
		Serial.print("\tM2:\t");
		Serial.println(motor_value_float[1]);
*/
// blink LED to indicate activity
		blinkState = !blinkState;
		digitalWrite(LED_PIN, blinkState);
	}
}

int parse_input(String input_string)
{
	//Turn on power supply
	if(input_string == "PSU_on" || input_string == "on")
	{
		psuState = 1;
		digitalWrite(psuPin,LOW);
		delay(3000);
		previous_time = micros()/100;
	}
	//Turn off power supply
	else if(input_string == "PSU_off" || input_string == "off")
	{
		psuState = 0;
		digitalWrite(psuPin,HIGH);
	}
	//Set coefficients for PID loop
	else if((input_string.substring(0,1) == "p") || (input_string.substring(0,1) == "i") || (input_string.substring(0,1) == "d"))
	{
		float tempCoefficient = 0;
		int pointPos = 0;
		int foundPoint = 0;
	
		//Parse each integer, storing in tempCoefficient
		for(int i=0; i<(input_string.length()-2); i++)
		{
			if((input_string.charAt(2+i) <= '9') && (input_string.charAt(2+i) >= '0')) tempCoefficient = 10*tempCoefficient + (input_string.charAt(2+i)-48);
			else if(input_string.charAt(2+i) == '.')
			{
				pointPos = i;
				foundPoint = 1;
			}
			else return 1;
		}
		
		//If decimal point present, divide by relevant power of ten
		if(foundPoint == 1) tempCoefficient /= pow(10,input_string.length()-3-pointPos);
		
		//Update relevant coefficient
		if(input_string.substring(0,1) == "p") pidConstants[0] = tempCoefficient;
		else if(input_string.substring(0,1) == "i") pidConstants[1] = tempCoefficient;
		else if(input_string.substring(0,1) == "d") pidConstants[2] = tempCoefficient;
	}
	//Set defaultSpeed
	else if(input_string.substring(0,1) == "t")
	{
		float tempCoefficient = 0;
		int pointPos = 0;
		int foundPoint = 0;
	
		//Parse each integer, storing in tempCoefficient
		for(int i=0; i<(input_string.length()-2); i++)
		{
			if((input_string.charAt(2+i) <= '9') && (input_string.charAt(2+i) >= '0')) tempCoefficient = 10*tempCoefficient + (input_string.charAt(2+i)-48);
			else if(input_string.charAt(2+i) == '.')
			{
				pointPos = i;
				foundPoint = 1;
			}
			else return 1;
		}
		
		//If decimal point present, divide by relevant power of ten
		if(foundPoint == 1) tempCoefficient /= pow(10,input_string.length()-3-pointPos);
		
		defaultSpeed = tempCoefficient;
	}
	//Reset PID integral values
	else if(input_string == "reset" || input_string == "r")
	{
		integral[0] = 0;
		integral[1] = 0;
	}
	else return 1;
	
	return 0;
}

int powint(int number,int exponent)
{
	if(number == 0) return 0;
	if(exponent == 0) return 1;
	
	int result = 1;
	for(int i=0; i < exponent; i++) result *= number;
	
	return result;
}

void updateMotors(const float *pidConstants,float *motor_value_float,const float* desired_angles,fifo<uint32_t>& timeArray,fifo<float>* angleArray,float *integral)
{
	float error[2]; //create array to hold error
	float change[2]; //create array to hold amount of change needed
	float derivative[2];
	uint32_t time_elapsed = timeArray[timeArray.size()-1] - timeArray[timeArray.size()-2]; // calculate time since last calculation
	
	float a = 0;
	float b = 0;
	float c = 0;
	//fitParabola(timeArray,angleArray[0],a,b,c);
	for(int i=0; i<2; i++)
	{
		error[i] = angleArray[2][angleArray[2].size()-1] - desired_angles[i]; //calculate current error
		integral[i] += error[i]*time_elapsed; //update value of integral
		derivative[i] = (angleArray[2][angleArray[2].size()-1] - angleArray[2][angleArray[2].size()-2])/time_elapsed;
		change[i] = pidConstants[0]*error[i] + pidConstants[1]*integral[i] + pidConstants[2]*derivative[i];
	}

	motor_value_float[0] = defaultSpeed - change[0];
	motor_value_float[1] = defaultSpeed + change[0];
	motor_value_float[2] = defaultSpeed;
	motor_value_float[3] = defaultSpeed;

	if(motor_value_float[0] < min_signal) motor_value_float[0] = min_signal;
	if(motor_value_float[0] > max_signal) motor_value_float[0] = max_signal;
	if(motor_value_float[1] < min_signal) motor_value_float[1] = min_signal;
	if(motor_value_float[1] > max_signal) motor_value_float[1] = max_signal;
	
	//if(motor_value_float[2] < min_signal) motor_value_float[2] = min_signal;
	//if(motor_value_float[2] < max_signal) motor_value_float[2] += change[0]/2;
	//if(motor_value_float[3] < min_signal) motor_value_float[3] = min_signal;
	//if(motor_value_float[3] < max_signal) motor_value_float[3] -= change[0]/2;
	
}

//Function to store a float (32-bit floating point) in a four byte array
void floatToByteArray(float &floatToConvert, uint8_t *byteArray)
{
	uint8_t* c = (uint8_t*)(&floatToConvert);
	
	byteArray[3] = *c;
	byteArray[2] = *(c+1);
	byteArray[1] = *(c+2);
	byteArray[0] = *(c+3);
}

//Fits parabola of form y=ax^2+bx+c to data
void fitParabola(fifo<uint32_t> &xVals,fifo<float> &yVals,float &a,float &b,float &c)
{
	//Check arrays have same length
	if(xVals.size() != yVals.size())
	{
		a = 0;
		b = 0;
		c = 0;
		return;
	}
	
	int numVals = xVals.size();
	
	//Scale our input values such that -1 < y < 1 and -1 < x < 1
	float xMax = xVals[0];
	float xMin = xVals[0];
	float yMax = yVals[0];
	float yMin = yVals[0];
	
	for(int i=0; i< numVals; i++)
	{
		if(xVals[i] < xMin) xMin = xVals[i];
		if(xVals[i] > xMax) xMax = xVals[i];
		
		if(yVals[i] < yMin) yMin = yVals[i];
		if(yVals[i] > yMax) yMax = yVals[i];
	}
	
	float xTrans = (xMax+xMin)/2.0;
	float yTrans = (yMax+yMin)/2.0;
	float xScale = (xMax-xMin)/2.0;
	float yScale = (yMax-yMin)/2.0;
	
	//Calculate different sums: x4 = sum(x[i]^4) for example
	float x4 = 0;
	float x3 = 0;
	float x2 = 0;
	float x = 0;
	float y = 0;
	float xy = 0;
	float x2y = 0;
	
	for(int i=0; i<numVals; i++)
	{
		x4 += pow((xVals[i]-xTrans)/xScale,4);
		x3 += pow((xVals[i]-xTrans)/xScale,3);
		x2 += pow((xVals[i]-xTrans)/xScale,2);
		x += (xVals[i]-xTrans)/xScale;
		y += (yVals[i]-yTrans)/yScale;
		xy += (xVals[i]-xTrans)/xScale*(yVals[i]-yTrans)/yScale;
		x2y += pow((xVals[i]-xTrans)/xScale,2)*(yVals[i]-yTrans)/yScale;
	}
	
	//Calculate the scaled coefficients
	float aScaled = ((x*y/numVals-xy)*(x3-x*x2/numVals)-(x2-pow(x,2)/numVals)*(x2*y/numVals-x2y))/((x4-pow(x2,2)/numVals)*(x2-pow(x,2)/numVals)-(x3-x*x2/numVals)*(x3-x*x2/numVals));
	float bScaled = -(aScaled*(x3-x*x2/numVals)+x*y/numVals-xy)/(x2-pow(x,2)/numVals);
	float cScaled = -(aScaled*x2+bScaled*x-y)/numVals;
	
	//Calculate the original coefficients
	a = yScale*aScaled/pow(xScale,2);
	b = (yScale*bScaled-2*a*xTrans*xScale)/xScale;
	c = yTrans + yScale*cScaled - a*pow(xTrans,2) - b*xTrans;
}
