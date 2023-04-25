//
// Created by Alex Curtis on 10/12/22.
//

#include "Control.h"
#include "Arduino.h"
#include <Wire.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
Pair<int> targetSpeed;              //!< Scaled PWM values given to motors.setSpeeds() each ranging from -400 to 400
Pair<long> counts, pastCounts;      //!< Left and right encoder readings (counts)
Zumo32U4Motors motors;        //!< Motor 2 is the right wheel
Zumo32U4Encoders encoders;    //!< Encoders for left and right wheels
Zumo32U4IMU imu;
float turnAngle = 0;

	// turnRate is the current angular rate of the gyro, in units of
	// 0.07 degrees per second.
	int16_t turnRate = 0;

	// This is the average reading obtained from the gyro's Z axis
	// during calibration.
	int16_t gyroOffset = 0;

	// This variable helps us keep track of how much time has passed
	// between readings of the gyro.
	uint16_t gyroLastUpdate = 0;

VelocityControl::VelocityControl() {}

void VelocityControl::drive(float targetPhi, float targetRho) {

	// Find current robot positions
	getPositions();
	sensorUpdate();

	// Update motorDif and motorSum with control() every CONTROL_SAMPLE_RATE ms
	if (millis()-startTime >= currentTime + CONTROL_SAMPLE_RATE) {

		// Adjust elapsed time
		currentTime += CONTROL_SAMPLE_RATE;

		motorSum = controlForward(currentDistance, targetRho);
		motorDif = controlAngle(-turnAngle * PI / 180, targetPhi * PI / 180);
		setMotors(motorDif, motorSum);
	}
	
	motors.setSpeeds(targetSpeed.L, targetSpeed.R);
}

void VelocityControl::startControl() {

	sensorSetup();
 	delay(500);
  	sensorReset();

	// Reset driving variables
	rhoOffset = 0;
	motorDif = 0;
	motorSum = 0;
	angleError = 0, distanceError = 0, pastErrorRho = 0, pastErrorPhi = 0;
	I_rho = 0, I_phi = 0;
	counts = {0,0};

	// Save current time
	currentTime = millis();
	startTime = millis();

	// Stop the motors
	motors.setSpeeds(0, 0);

}

/**
 * Reset controller when finished
 */
void VelocityControl::stopControl() {

	// Reset encoders
	encoders.getCountsAndResetRight();
	encoders.getCountsAndResetLeft();
	
	// Reset IMU
	sensorReset();

	motorDif = 0;
	motorSum = 0;
	angleError = 0, distanceError = 0, pastErrorRho = 0, pastErrorPhi = 0;
	I_rho = 0, I_phi = 0;

	// Set left and right motor speeds to 0
	motors.setSpeeds(0, 0);

} 

/**
 * Read current encoder counts and calculate currentAngle and currentDistance
 */
void VelocityControl::getPositions() {
	// Update encoder counts
	counts = {encoders.getCountsLeft(), encoders.getCountsRight()};

	// Find current robot positions
	// currentAngle = (RADIUS * RAD_CONVERSION * float(counts.L - counts.R)) / BASE;
	currentDistance = RADIUS * RAD_CONVERSION * float(counts.L + counts.R) * float(0.5); // Circumference = 2*PI*r

}

float VelocityControl::controlForward(float current, float desired) {

	float P = 0, D = 0, output = 0;

	// Calculate error
	distanceError = desired - current;

	// Calculate P component
	P = KP_RHO * distanceError;

	// Calculate I component
	I_rho += KI_RHO * float(CONTROL_SAMPLE_RATE / 1000.0) * distanceError;

	// Calculate D component
	if (currentTime > 0) {
		D = (distanceError - pastErrorRho) / float(CONTROL_SAMPLE_RATE / 1000.0);
		pastErrorRho = distanceError;
		D *= KD_RHO;
	} else D = 0;

	// Calculate controller output
	output = P + I_rho + D;

	// Make sure the output is within [-400, 400]
	if(output > 400) output = 400;
	if(output < -400) output = -400;

	return output;
}

float VelocityControl::controlAngle(float current, float desired) {

	float P = 0, D = 0, output = 0;
	
	// Calculate error
	angleError = desired - current;
	
	// Calculate P component
	P = KP_PHI * angleError;
	
	// Calculate I component
	if(angleError > 90*(PI/180) || angleError < -90*(PI/180)) {
		I_phi = 0;
	} else {
		I_phi += KI_PHI * float(CONTROL_SAMPLE_RATE / 1000.0) * angleError;
	}

	// Calculate D component
	if (currentTime > 0) {
		D = (angleError - pastErrorPhi) / float(CONTROL_SAMPLE_RATE / 1000.0);
		pastErrorPhi = angleError;
		D *= KD_PHI;
	} else D = 0;

	// Calculate controller output
	output = P + I_phi + D;
	
	// Make sure the output is within [0, 400]
	if(output > 400) output = 400;
	else if(output < -400) output = -400;

	// If the angle error is less than 1 degree, stop the motors
	if(abs(angleError)*180/PI < 1) {
		output = 0;
	}

	return output;
}

void VelocityControl::setMotors(float diff, float sum) const {
	Pair<float> target = {0,0}; 		//!< Motor PWM outputs

	target.R = (sum - diff) / float(2.0);
	target.L = (sum + diff) / float(2.0);

	// Make sure the speeds are within [-400, 400]
	if(target.R > MAX_SPEED) target.R = MAX_SPEED;
	if(target.L > MAX_SPEED) target.L = MAX_SPEED;
	if(target.R < -MAX_SPEED) target.R = -MAX_SPEED;
	if(target.L < -MAX_SPEED) target.L = -MAX_SPEED;

	// Update the targetSpeed variable
	targetSpeed = {int(target.L),int(target.R)};
}

bool VelocityControl::isDone() {
	bool done = false;
	// If the errors are low enough for long enough, return true

	// Every MIN_SETTLING_TIME milliseconds
	if (millis()-startTime >= lastTime + MIN_SETTLING_TIME) {

		lastTime += MIN_SETTLING_TIME;

		// If the encoders haven't changed
		if(counts.L == pastCounts.L && counts.R == pastCounts.R) {
			// The robot is done moving, or isn't moving fast enough for us to wait.
			done = true;
		}

		// Save position
		pastCounts = counts;
	}

	return done;
}

bool VelocityControl::isDoneRotating() {
	bool done = false;
	// If the errors are low enough for long enough, return true

	// Every MIN_SETTLING_TIME milliseconds
	if (millis()-startTime >= lastTime + 1000) {

		lastTime += 1000;

		// If the encoders haven't changed
		if(counts.L == pastCounts.L && counts.R == pastCounts.R) {
			// The robot is done moving, or isn't moving fast enough for us to wait.
			done = true;
		}

		// Save position
		pastCounts = counts;
	}

	return done;
}

void VelocityControl::sensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void VelocityControl::sensorUpdate()
{
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += ((float)imu.g.z - gyroOffset) * 70 * dt / 1000000000;
}

/* This should be called in setup() to enable and calibrate the
gyro.  It uses the display, yellow LED, and button A.  While the
display shows "Gyro cal", you should be careful to hold the robot
still.

The digital zero-rate level of the gyro can be as high as
25 degrees per second, and this calibration helps us correct for
that. */
void VelocityControl::sensorSetup() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  // Turn on the yellow LED in case the display is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  sensorReset();
}
