#include <Arduino.h>
#include <Control.h>
#include "Wire.h"
#include "Zumo32U4Encoders.h"
#include "Zumo32U4Motors.h"

int newRho = 24, newPhi = 360;
bool done = false;

Control control;

void setup() {
	// Begin serial communication
	Serial.begin(9600);
}

void loop() {

//	countsLocal = {encodersLocal.getCountsLeft(), -encodersLocal.getCountsRight()};
//	Serial.print("R: "); Serial.print(countsLocal.R);
//	Serial.print("\tL: "); Serial.println(countsLocal.L);

	done = control.drive(newPhi,newRho);

	if(done){

		newRho = -1*newRho;
		newPhi = -1*newPhi;
	}

}





