//
// Created by Alex Curtis on 10/12/22.
//
#include <Arduino.h>
#include <Zumo32U4.h>
#include "Zumo32U4Encoders.h"
#include "Zumo32U4Motors.h"
#include "Zumo32U4IMU.h"
#ifndef Control_h
#define Control_h


template<typename T>
struct Pair {
	T L;
	T R;
	Pair operator+(const T &a) const { return Pair<T>({T(L) + a, T(R) + a}); };
	Pair operator+(const Pair<T> &a) const { return Pair<T>({T(L) + a.L, T(R) + a.R}); };
	Pair operator-(const T &a) const { return Pair<T>({T(L) - a, T(R) - a}); };
	Pair operator-(const Pair<T> &a) const { return Pair<T>({T(L) - a.L, T(R) - a.R}); };
	Pair operator*(const T &a) const { return Pair<T>({T(L) * a, T(R) * a}); };
	Pair operator*(const Pair<T> &a) const { return Pair<T>({T(L) * a.L, T(R) * a.R}); };
	Pair operator/(const T &a) const { return Pair<T>({T(L) / a, T(R) / a}); };
	Pair operator/(const Pair<T> &a) const { return Pair<T>({T(L) / a.L, T(R) / a.R}); };
};

class VelocityControl {
public:
	//Default constructor
	VelocityControl();
	const Pair<float> MIN_SPEED = {81.879180,80.635212};    //!< Minimum scaled PWM //TODO implement this
	unsigned long MIN_SETTLING_TIME = 2000;					//!< Time in ms to wait for motors to settle // TODO adjust this if needed
	const float CPR = 909.7;                                 //!< Total encoder counts per revolution (CPR) of motor shaft = 3200 counts/rot
	const float RADIUS = 0.7689; //!< Measured radius of wheels in inches
	const float BASE = 4.0;                                //!< Distance between center of wheels in inches
	const float RAD_CONVERSION = float(2.0 * PI) / CPR;     //!< Scalar to convert counts to radians

	// public variables
	const float KP_RHO = 100, KI_RHO = 30, KD_RHO = 0.000000; //!< Rho controller constants
	const float KP_PHI = 200, KI_PHI = 80, KD_PHI = 0.000000; //!< Phi controller constants
	float currentDistance = 0;                           //!< current and target distances in inches
	float currentAngle = 0;                           //!< current and target angles in radians

	// public methods
	void startControl(); //!< setup
	
	void getPositions();
	void drive(float targetPhi, float targetRho);
	float controlForward(float current, float desired);
	float controlAngle(float current, float desired);
	void setMotors(float diff, float sum) const;

	void stopControl();

	// Status
	bool isDone();
	bool isDoneRotating();
	
	// IMU functions
    void sensorSetup();
	void sensorUpdate();
	void sensorReset();

	const long CONTROL_SAMPLE_RATE = 10;                     //!< Controller sample rate in ms
	const int MAX_SPEED = 200;
	float rhoOffset = 0;                                    //!< Contains initial forward counts after rotating
	float motorDif = 0, motorSum = 0;                               //!< Parameters for speed control. motorDif [-400,400] and motorSum [-400, 400]
	float angleError = 0, distanceError = 0, pastErrorRho = 0, pastErrorPhi = 0;        //!< Variables used in calculating control output
	float I_rho = 0, I_phi = 0;                             //!< Integral controller accumulations
	unsigned long currentTime = 0, startTime = 0, lastTime = 0;           //!< For creating a discrete time controller
	
	bool firstRho = true;               //!< Flag for accurately determining forward counts after rotating

};
#endif //Control_h
