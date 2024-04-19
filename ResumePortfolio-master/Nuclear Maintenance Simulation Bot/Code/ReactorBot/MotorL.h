//**************************************************************/
// TITLE
//**************************************************************/

// MotorL.h
// Namespace for ReactorBot left drive motor.
// RBE-2001 A17 Team 7

#pragma once
#include "DcMotor.h"

//**************************************************************/
// NAMESPACE DEFINITION
//**************************************************************/

namespace MotorL {

	// Arduino Pin Settings
	const uint8_t PIN_ENABLE = 31;
	const uint8_t PIN_FORWARD = 4;
	const uint8_t PIN_REVERSE = 5;
	const uint8_t PIN_ENCODER_A = 18;
	const uint8_t PIN_ENCODER_B = 19;

	// Motor Physical Properties
	const float TERMINAL_VOLTAGE = 12.0;
	const float ENCODER_CPR = 3200.0;

	// Motor object
	DcMotor motor(
		TERMINAL_VOLTAGE,
		PIN_ENABLE,
		PIN_FORWARD,
		PIN_REVERSE,
		PIN_ENCODER_A,
		PIN_ENCODER_B,
		ENCODER_CPR);

	// Encoder ISRs
	void interruptA() { motor.interruptA(); }
	void interruptB() { motor.interruptB(); }

	// Motor initialization (call in setup)
	void setup() {
		motor.setup();
		motor.enable();
		attachInterrupt(
			motor.getInterruptA(),
			interruptA,
			CHANGE);
		attachInterrupt(
			motor.getInterruptB(),
			interruptB,
			CHANGE);
	}
}
