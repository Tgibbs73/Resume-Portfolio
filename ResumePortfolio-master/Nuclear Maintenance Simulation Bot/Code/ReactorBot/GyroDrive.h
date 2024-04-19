//**************************************************************/
// TITLE
//**************************************************************/

// GyroDrive.h
// Namespace for ReactorBot gyroscopic drive control system.
// RBE-2001 A17 Team 7

#pragma once
#include "Bno055.h"
#include "PidController.h"
#include "MotorL.h"
#include "MotorR.h"

//**************************************************************/
// NAMESPACE DEFINITION
//**************************************************************/

namespace GyroDrive {

	Bno055 imu(trb); // IMU with dot on Top Right Back of chip
	double h0 = 0.0; // Absolute IMU heading offset

	//!b Initializes IMU (call in setup)
	void setup() {
		imu.begin();
		h0 = imu.heading();
	}

	//!b Returns robot heading (rad)
	float heading() {
		return imu.heading() - h0;
	}

	// PID controllers will reset if not used for this time
	const float PID_RESET_TIME = 0.1;

	// Robot Heading PID Controller
	// Input: Heading (rad)
	// Output: Differential motor voltage (V)
	const float ANGLE_VMAX = 8.0;
	const float ANGLE_KP = 3.0;
	const float ANGLE_KI = 1.0;
	const float ANGLE_KD = 0.0;
	PidController anglePid(
		ANGLE_KP,
		ANGLE_KI,
		ANGLE_KD,
		-ANGLE_VMAX,
		+ANGLE_VMAX,
		PID_RESET_TIME);

	// PID turns robot to given absolute heading (rad)
	// If stabilized, returns true and brakes motors
	bool setAngle(float h) {
		float err;
		float hc = heading();
		if(h <= PI) {
			if(hc <= h + PI) err = h - hc;
			else err = h + TWO_PI - hc;
		} else {
			if(hc <= h - PI) err = h - TWO_PI - hc;
			else err = h - hc;
		}
		float vdd = anglePid.update(err);
		MotorL::motor.setVoltage(+vdd);
		MotorR::motor.setVoltage(-vdd);
		if(anglePid.isStabilized(0.05, 0.01)) {
			MotorL::motor.brake();
			MotorR::motor.brake();
			return true;
		} else
			return false;
	}

	// Angular Velocity PID Controller
	// Input: Robot angular velocity (rad/s)
	// Output: Differential motor voltage (V)
	const float VEL_VMAX = 12.0;
	const float VEL_KP = 1.5;
	const float VEL_KI = 30.0;
	const float VEL_KD = 0.0;
	PidController velPid(
		VEL_KP,
		VEL_KI,
		VEL_KD,
		-VEL_VMAX,
		+VEL_VMAX,
		PID_RESET_TIME);

	// PID sets robot angular velocity and linear drive voltage
	// w is target angular velocity (rad/s)
	// v is straight line drive voltage (V)
	void setVelocity(float w, float v = 0) {
		float vdd = velPid.update(w - imu.gZ());
		MotorL::motor.setVoltage(v - vdd);
		MotorR::motor.setVoltage(v + vdd);
	}

	// Resets all PID controllers in namespace
	void resetPids() {
		anglePid.reset();
		velPid.reset();
	}
}
