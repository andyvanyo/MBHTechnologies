#include <Arduino.h>
#include <Control.h>
#include "Wire.h"
#include <Zumo32U4.h>
// Zumo32U4Encoders encoders;

int newRho = 0, newPhi = 360;
bool done = false;
float angle = 0;
int32_t DESIRED = 90;

// int L = 0, R = 0;
// float theta = 0;
// float distance = 0;

// const float CPR = 909.7;                                 //!< Total encoder counts per revolution (CPR) of motor shaft = 3200 counts/rot
// const float RADIUS = 0.7689;                            //!< Measured radius of wheels in inches
// const float BASE = 4.0;                                //!< Distance between center of wheels in inches
// const float RAD_CONVERSION = float(2.0 * PI) / CPR;     //!< Scalar to convert counts to radians


// Control control;

// void setup() {
// 	// Begin serial communication
// 	Serial.begin(9600);
	
// }

// void loop() {
	
// 	// L = encoders.getCountsLeft();
// 	// R = encoders.getCountsRight();
// 	// theta = (RADIUS * RAD_CONVERSION * float(L - R)) / BASE;
// 	// distance = RADIUS * RAD_CONVERSION * float(L + R) * float(0.5);

// 	// Serial.print("L: ");
// 	// Serial.print(L);
// 	// Serial.print("\t");
// 	// Serial.print("R: ");
// 	// Serial.print(R);
// 	// Serial.print("\t");
// 	// Serial.print("Avg: ");
// 	// Serial.print((L + R) * 0.5);
// 	// Serial.print("\t");
// 	// Serial.print("Angle: ");
// 	// Serial.print(theta * 180 / PI);
// 	// Serial.print("\t");
// 	// Serial.print("Distance: ");
// 	// Serial.println(distance, 4);
// 	done = control.drive(newPhi, newRho);
	
// 	if(done) {
// 		Serial.println("Done");
// 		newRho *= -1;
// 		newPhi *= -1;
// 		done = false;
// 	}

// }

#include <Wire.h>
#include <Zumo32U4.h>

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const int16_t maxSpeed = 400;

// Change next line to this if you are using the older Zumo 32U4
// with a black and green LCD display:
// Zumo32U4LCD display;
Zumo32U4OLED display;

Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
Zumo32U4IMU imu;

#include "TurnSensor.h"

void setup()
{
  turnSensorSetup();
  delay(500);
  turnSensorReset();

  display.clear();
  display.print(F("Try to"));
  display.gotoXY(0, 1);
  display.print(F("turn me!"));
}

void loop()
{
  // Read the gyro to update turnAngle, the estimation of how far
  // the robot has turned, and turnRate, the estimation of how
  // fast it is turning.
  turnSensorUpdate();

  // Calculate the motor turn speed using proportional and
  // derivative PID terms.  Here we are a using a proportional
  // constant of 56 and a derivative constant of 1/20.
  int32_t error = DESIRED - ((((int32_t)turnAngle >> 16) * 360) >> 16) ;
  Serial.print("Turn Angle: ");
  Serial.print((((int32_t)turnAngle >> 16) * 360) >> 16, 5);
  Serial.print("\t");
  Serial.print("e: ");
  Serial.println(error,5);


  // Constrain our motor speeds to be between
  // -maxSpeed and maxSpeed.
//   error = constrain(turnSpeed, -maxSpeed, maxSpeed);

//   motors.setSpeeds(-turnSpeed, turnSpeed);
}

