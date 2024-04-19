//**************************************************************/
// TITLE
//**************************************************************/

// LineFollower.h
// Namespace for ReactorBot line follower.
// RBE-2001 A17 Team 7

#pragma once
#include "Qtr8.h"
#include "GyroDrive.h"

//**************************************************************/
// NAMESPACE DEFINITION
//**************************************************************/

namespace LineFollower {

	// Line Follower Parameters
	const float DRIVE_VOLTAGE = 4.0; // Default (V)
	const float ANGULAR_SPEED = 1.0; // Maximum (rad/s)

	// QTR-8 Analog Line Sensor
	const int THRESHOLD_WHITE = 100; // 10-bit ADC value
	const int THRESHOLD_BLACK = 700; // 10-bit ADC value
	const uint8_t PINS[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };
	Qtr8 sensor(PINS);

	// Initializes light sensor (call in setup)
	void setup() {
		sensor.setup();
		sensor.setWhiteThreshold(THRESHOLD_WHITE);
		sensor.setBlackThreshold(THRESHOLD_BLACK);
	}

	// Line Follower PID Controller
	// Input: Line displacement (cm)
	// Output: Angular velocity (rad/s)
	const float KP = 1.0;
	const float KI = 0.0;
	const float KD = 0.0;
	PidController pid(KP, KI, KD,
		-ANGULAR_SPEED,
		+ANGULAR_SPEED);

	// Line follows forward with given drive voltage
	// Default drive voltage is DRIVE_VOLTAGE parameter
	void drive(float v = DRIVE_VOLTAGE) {
		float w = pid.update(sensor.linePos());
		GyroDrive::setVelocity(w, v);
	}

	// Memory for racking line intersections
	bool blackBefore = false;

	// Returns true on black line intersection (rising edge)
	bool hitIntersection() {
		bool blackNow = sensor.onBlack();
		bool intersect = (blackNow && !blackBefore);
		blackBefore = blackNow;
		return intersect;
	}

	// Resets all PID controllers in namespace
	void resetPids() {
		pid.reset();
	}
}
