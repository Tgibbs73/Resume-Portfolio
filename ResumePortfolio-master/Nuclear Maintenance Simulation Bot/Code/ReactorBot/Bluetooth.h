//**************************************************************/
// TITLE
//**************************************************************/

// Bluetooth.h
// Namespace for ReactorBot Bluetooth communication.
// RBE-2001 A17 Team 7

#pragma once
#include "ReactorComms.h"
#include "MotorL.h"
#include "MotorR.h"
#include "Arm.h"
#include "GyroDrive.h"
#include "LineFollower.h"
#include "Timer.h"

//**************************************************************/
// NAMESPACE DEFINITION
//**************************************************************/

namespace Bluetooth {

	Timer timer; // Counts time between heartbeats
	ReactorComms com(Serial3); // Reactor communication object

	// Initializes Bluetooth and heartbeat (call in setup).
	void setup() {
		com.init();
		timer.tic();
	}

	// Performs ReactorBot Bluetooth actions (call in loop).
	void loop(int radLevel) {

		// Check Bluetooth messages
		com.update();

		// Enable or disable drive
		if(com.getRobotEnabled()) {
			MotorL::motor.enable();
			MotorR::motor.enable();
			Arm::motor.enable();
		} else {
			MotorL::motor.disable();
			MotorR::motor.disable();
			Arm::motor.disable();
			GyroDrive::resetPids();
			LineFollower::resetPids();
			Arm::resetPids();
		}

		// Send heartbeat and radiation alerts
		if(timer.hasElapsed(1.0)) {
			timer.tic();
			com.sendHeartBeat();
			switch(radLevel) {
				case 3:
					com.sendRadAlert(RADIATION_HI); break;
				case 2:
					com.sendRadAlert(RADIATION_LO); break;
				default: break;
			}
		}
	}
}
