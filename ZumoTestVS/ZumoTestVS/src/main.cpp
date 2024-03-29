#include <Arduino.h>
#include <Control.h>
#include "Wire.h"
#include <Zumo32U4.h>

int newRho = 0, newPhi = 0;
int sum = 0;
int diff = 0;
bool done = false;

void readChar();
void startExperiment();
char receivedChar;
bool newData = false;

Control control;
Zumo32U4Motors Motors;

//!< Reads a byte sent over serial from Matlab
void readChar() {
	if (Serial.available() > 0) {
		receivedChar = Serial.read();
		newData = true;
	}
}

void startExperiment() {
	if(newData) {
		if(receivedChar == 'f'){
			newRho = 12;
			newPhi = 0;
		} else if(receivedChar == 'b') {
			newRho = -12;
			newPhi = 0;
		} else if(receivedChar == 'l') {
			newPhi = -90;
			newRho = 0;
		} else if(receivedChar == 'r') {
			newPhi = 90;
			newRho = 0;
		}
		else if(receivedChar == 's') {
			newPhi = 0;	
			newRho = 0;
		}
		newData = false;
		Serial.println("Received");
	}
}

void setup() {
	// Begin serial communication
	Serial.begin(9600);
	while(!Serial); // Wait for serial connection
	Serial.println("<Zumo is ready>");
	Motors.setSpeeds(0, 0);
}

void loop() {
	readChar();
	startExperiment();
	
}


















// #include <Wire.h>
// #include <Zumo32U4.h>

// // This is the maximum speed the motors will be allowed to turn.
// // A maxSpeed of 400 lets the motors go at top speed.  Decrease
// // this value to impose a speed limit.
// const int16_t maxSpeed = 400;

// // Change next line to this if you are using the older Zumo 32U4
// // with a black and green LCD display:
// // Zumo32U4LCD display;
// Zumo32U4OLED display;

// Zumo32U4ButtonA buttonA;
// Zumo32U4Motors motors;
// Zumo32U4IMU imu;

// #include "TurnSensor.h"

// void setup()
// {
//   turnSensorSetup();
//   delay(500);
//   turnSensorReset();

//   display.clear();
//   display.print(F("Try to"));
//   display.gotoXY(0, 1);
//   display.print(F("turn me!"));
// }

// void loop()
// {
//   // Read the gyro to update turnAngle, the estimation of how far
//   // the robot has turned, and turnRate, the estimation of how
//   // fast it is turning.
//   turnSensorUpdate();

//   // Calculate the motor turn speed using proportional and
//   // derivative PID terms.  Here we are a using a proportional
//   // constant of 56 and a derivative constant of 1/20.



//   // Constrain our motor speeds to be between
//   // -maxSpeed and maxSpeed.
// //   error = constrain(turnSpeed, -maxSpeed, maxSpeed);

// //   motors.setSpeeds(-turnSpeed, turnSpeed);
// }

