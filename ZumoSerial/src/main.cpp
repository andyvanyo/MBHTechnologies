#include <Arduino.h>
#include <Control.h>
#include "Wire.h"
#include <Zumo32U4.h>

volatile float newRho = 0, newPhi = 0;
int sum = 0;
int diff = 0;
bool fineControl = false;

void readChar();
void parseSerial();
char receivedChar;
bool newData = false;
const long PRINT_INTERVAL = 2000;
long printStartTime = millis();
long printTime = 0;

VelocityControl control;
Zumo32U4Motors Motors;

void readChar() {
	if (Serial.available() > 0) {
		receivedChar = Serial.read();
		newData = true;
	}
}
volatile float rhoIncrement = 2;
volatile float phiIncrement = 45;

void parseSerial() {
	if(newData) {
		if(receivedChar == 'f') {
			rhoIncrement = 1;
			phiIncrement = 10;
		} else if(receivedChar == 'r') {
			rhoIncrement = 2;
			phiIncrement = 45;
		} else if(receivedChar == 'w') {
			newRho += rhoIncrement;
		} else if(receivedChar == 's') {
			newRho -= rhoIncrement;
		} else if(receivedChar == 'a') {
			newPhi -= phiIncrement;
		} else if(receivedChar == 'd') {
			newPhi += phiIncrement;
		} else if(receivedChar == 'x') {
			newRho = 0;
			newPhi = 0;
			control.stopControl();
		}
		newData = false;
		Serial.println("Received");
	}
}

void setup() {
	Serial.begin(9600);
	while(!Serial); // Wait for serial connection
	control.startControl();
	// Serial.println("<Zumo is ready>");
	printStartTime = millis();
	Motors.setSpeeds(0, 0);
}



void loop() {
	readChar();
	parseSerial();

	control.drive(newPhi, newRho);
	
	if (millis()-printStartTime >= printTime + PRINT_INTERVAL) {

		// Adjust elapsed time
		printTime += PRINT_INTERVAL;

		Serial.print("angleE: ");
		Serial.print(control.angleError*180/PI);
		Serial.print("\t");
		Serial.print("targetRho: ");
		Serial.print(newRho);
		Serial.print("\t");
		Serial.print("currentRho: ");
		Serial.print(control.currentDistance);
		Serial.print("\t");
		Serial.print("rhoE: ");
		Serial.println(control.distanceError);
	}

	
}

