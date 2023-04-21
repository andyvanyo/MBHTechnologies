#include <Arduino.h>
#include <Control.h>
#include "Wire.h"
#include "Zumo32U4Encoders.h"


int newRho = 1, newPhi = 180;
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
	done = control.drive(newPhi, newRho);
	if(done) {
		Serial.println("Done");
		newRho *= -1;
		newPhi *= -1;
		done = false;
	}

	

}





