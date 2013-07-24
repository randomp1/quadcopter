#include <Servo.h>
#include <stdlib.h>

//Set up serial variables
String incoming_string;

//PSU control data
int psuPin = 6;		//PSU pin
int psuState = 0;	//PSU on or off

//Motor control data
const int motor_pin[4] = {11,10,9,8};			//Pin assignment
Servo motor[4];									//Servo objects for motor
const int min_signal = 40;						//Minimum signal required to turn on motor
const int max_signal = 50;						//Maximum allowed signal (for safety!)
float motor_value_float[4] = {0.0,0.0,0.0,0.0};	//Array of motor values

//Function to parse input strings
int parse_input(String input_string);

void setup()
{
	Serial.begin(115200);
	
	//Set motor pins as output and initialise the servo communications. Set the motors to be off!
	for(int i=0; i < 4; i++) pinMode(motor_pin[i],OUTPUT);
	for(int i=0; i < 4; i++) motor[i].attach(motor_pin[i],1000,2000); //this should really be 1100, 1800, but the ESC doesn't seem to accept it
	for(int i=0; i < 4; i++) motor[i].write(0);
	
	//Set PSU power signal pin (for turning it on)
	pinMode(psuPin,OUTPUT);
	digitalWrite(psuPin,HIGH);
}

void loop()
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
	
	if(psuState == 1) motor[0].write(motor_value_float[0]);
}

int parse_input(String input_string)
{
	//Turn on power supply
	if(input_string == "PSU_on" || input_string == "on")
	{
		psuState = 1;
		digitalWrite(psuPin,LOW);
		delay(5000);
	}
	//Turn off power supply
	else if(input_string == "PSU_off" || input_string == "off")
	{
		psuState = 0;
		digitalWrite(psuPin,HIGH);
	}
	else if(input_string.substring(0,1) == "s")
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
		
		//Update motor speed
		motor_value_float[0] = tempCoefficient;
	}
	else return 1;
	
	return 0;
}
