#include <Arduino.h>
#include <Control.h>
#include "Wire.h"
#include <Zumo32U4.h>

int newRho = 24, newPhi = 90;
bool done = false;

Control control;

void setup() {
	// Begin serial communication
	Serial.begin(9600);
}

void loop() {
	
	done = control.drive(newPhi, newRho);
	if(done) {
		Serial.println("Done");
		newPhi += 90;
		done = false;
	}
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

