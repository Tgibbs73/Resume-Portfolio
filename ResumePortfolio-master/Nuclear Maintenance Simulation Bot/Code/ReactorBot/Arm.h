//**************************************************************/
// TITLE
//**************************************************************/

// Arm.h
// Namespace for ReactorBot arm.
// RBE-2001 A17 Team 7

#pragma once
#include "DcMotor.h"
#include "PidController.h"

//**************************************************************/
// NAMESPACE DEFINITION
//**************************************************************/

namespace Arm {

	// Arduino Pin Settings
	const uint8_t PIN_ENABLE = 11;
	const uint8_t PIN_FORWARD = 12;
	const uint8_t PIN_REVERSE = 13;
	const uint8_t PIN_ANGLE_POT = A15;

	// Motor Physical Properties
	const float TERMINAL_VOLTAGE = 12.0;

	// Motor object
	DcMotor motor(
		TERMINAL_VOLTAGE,
		PIN_ENABLE,
		PIN_FORWARD,
		PIN_REVERSE,
		0, 0, 1); // Ignored settings

	// Motor initialization (call in setup)
	void setup() {
		motor.setup();
		motor.enable();
		pinMode(PIN_ANGLE_POT, INPUT);
	}

	// Returns arm angle (10-bit ADC)
	int getAngle() {
		return analogRead(PIN_ANGLE_POT);
	}

	// Arm PID Setpoints
	const int ANGLE_BACK = 543;
	const int ANGLE_TUBE = 344;
	const int ANGLE_PREP_1 = 294;
	const int ANGLE_PREP_2 = 305;
	const int ANGLE_PICKUP = 69;
	const int ANGLE_DROPOFF = 110;

	// Arm Angle PID Controller
	// Input: Potentiometer reading (10-bit ADC)
	// Output: Motor voltage (V)
	const float PID_KP = 0.015;
	const float PID_KI = 0.011;
	const float PID_KD = 0.0;
	const float RESET_TIME = 0.1;
	PidController pid(
		PID_KP,
		PID_KI,
		PID_KD,
		-TERMINAL_VOLTAGE,
		+TERMINAL_VOLTAGE,
		RESET_TIME);

	// PID rotates arm to given setoint (1 iteration)
	// Returns true and brakes motor if arm is stable at setpoint
	bool setAngle(int setPoint) {
		motor.setVoltage(pid.update(setPoint - getAngle()));
		if(pid.isStabilized(5.0, 1.0)) {
			motor.brake();
			return true;
		} else
			return false;
	}

	// Resets all PID controllers in namespace
	void resetPids() {
		pid.reset();
	}
}
