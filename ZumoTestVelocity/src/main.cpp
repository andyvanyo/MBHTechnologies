#include <Arduino.h>
#include <Control.h>
#include "Wire.h"
#include "Zumo32U4Encoders.h"
#include "Zumo32U4Motors.h"

int newRho = 24, newPhi = 0;
bool done = false;
Zumo32U4Motors Motors;        //!< Motor 2 is the right wheel

Control control;

void setup() {
	// Begin serial communication
	Serial.begin(9600);
}

void loop() {

//	countsLocal = {encodersLocal.getCountsLeft(), -encodersLocal.getCountsRight()};
//	Serial.print("R: "); Serial.print(countsLocal.R);
//	Serial.print("\tL: "); Serial.println(countsLocal.L);

int speed = 200;
	// done = control.drive(newPhi, newRho);
	Motors.setSpeeds(speed, speed);

	

}





