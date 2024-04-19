//**************************************************************/
// TITLE
//**************************************************************/

// Gripper.h
// Namespace for ReactorBot gripper servo.
// RBE-2001 A17 Team 7

#pragma once
#include "Servo.h"
#include "Timer.h"

//**************************************************************/
// NAMESPACE DEFINITION
//**************************************************************/

namespace Gripper {

	// Constants
	const uint8_t PIN_SERVO = 10; // Servo PWM pin
	const float GRIP_TIME = 1.0;  // Time of grip (sec)
	const int ANGLE_OPEN = 60;    // Servo control angle
	const int ANGLE_CLOSED = 144; // Servo control angle

	Servo gripper;
	Timer timer;

	// Initializes gripper (call in setup).
	void setup() {
		gripper.attach(PIN_SERVO);
	}

	// Opens gripper.
	void open() {
		gripper.write(ANGLE_OPEN);
		timer.tic();
	}

	// Closes gripper.
	void close() {
		gripper.write(ANGLE_CLOSED);
		timer.tic();
	}

	// Returns true when the gripper is done gripping.
	bool ready() {
		return timer.hasElapsed(GRIP_TIME);
	}
}
