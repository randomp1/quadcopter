#include <Servo.h>

//Analog port to use
const int inputPin = 3;
bool interrupt_flag = 0;
const int period =  1000000;

//Set up serial variables
String incoming_string;

//PSU control data
const int psuPin = 6;	//PSU pin
int psuState = 0;	//PSU on or off

//Motor control data
const int motor_pin[4] = {10,11,9,8};			//Pin assignment
Servo motor[4];						//Servo objects for motor
const int min_signal = 40;				//Minimum signal required to turn on motor
const int max_signal = 60;			        //Maximum allowed signal (for safety!)
float motor_value_float[4] = {0.0,0.0,0.0,0.0};	        //Array of motor values

void flagger()
{
	interrupt_flag = 1;
}

void setup()
{
	Serial.begin(115200);
	pinMode(inputPin,INPUT);
	attachInterrupt(1,flagger,FALLING);
	
	//Set motor pins as output and initialise the servo communications. Set the motors to be off!
	for(int i=0; i < 4; i++) pinMode(motor_pin[i],OUTPUT);
	for(int i=0; i < 4; i++) motor[i].attach(motor_pin[i],1000,2000); //this should really be 1100, 1800, but the ESC doesn't seem to accept it
	for(int i=0; i < 4; i++) motor[i].write(0);
	
	//Set PSU power signal pin (for turning it on)
	pinMode(psuPin,OUTPUT);
	digitalWrite(psuPin,LOW);
	delay(10000);
	
	motor[0].write(40);
}

void loop()
{
        motor[0].write(50.0+10.0*sin(2.0*PI/period*float(micros())));
	if (interrupt_flag == 1)
	{
		//send time
		Serial.println(micros());
		interrupt_flag = 0;
	}
}
