//**************************************************************/
// TITLE
//**************************************************************/

// StateMachine.h
// File describing ReactorBot operating state machine.
// RBE-2001 A17 Team 7

#pragma once

//**************************************************************/
// DEPENDENCIES
//**************************************************************/

// Included Libraries
#include "Arduino.h"
#include "LimitSwitch.h"

// High Level Control
#include "FieldPosition.h"
#include "Bluetooth.h"

// Physical Object Namespaces
#include "MotorL.h"
#include "MotorR.h"
#include "Arm.h"
#include "Gripper.h"
#include "IndicatorLed.h"

// Drive Controllers
#include "GyroDrive.h"
#include "LineFollower.h"

//**************************************************************/
// INTERNAL ODOMETRY
//**************************************************************/

// Field location
FieldPosition currentPos(2, 0);
FieldPosition targetPos = REACTOR_A;

// Field orientation
float targetHeading = 0;
const float HEADING_U = PI * 0.0 / 2.0; // Up
const float HEADING_R = PI * 1.0 / 2.0; // Right
const float HEADING_D = PI * 2.0 / 2.0; // Down
const float HEADING_L = PI * 3.0 / 2.0; // Left

// Arm orientation
int targetArmAngle = 0;

// Motor angle to inch VTC onto line intersection
const float VTC_INCH_ANGLE = 2.465;

//**************************************************************/
// STATE MACHINE
//**************************************************************/

// Current reactor being refueled
enum reactor_t {
	A = 1,
	B = 6
} reactor;

// Task for current reactor
enum task_t {
	TASK_EMPTY_REACTOR,
	TASK_FILL_STORAGE,
	TASK_GET_SUPPLY,
	TASK_FILL_REACTOR,
} task;

// State within current task
enum state_t {
	STATE_BEGIN,
	STATE_DECIDE_X,
	STATE_TURNTO_X,
	STATE_GOTO_X,
	STATE_PREP_DEPOSIT_1,
	STATE_PREP_DEPOSIT_2,
	STATE_APPROACH_REACTOR,
	STATE_INCH_X,
	STATE_DECIDE_Y,
	STATE_TURNTO_Y,
	STATE_GOTO_Y,
	STATE_DECIDE_ARM,
	STATE_ARM_FORWARD,
	STATE_DECIDE_GRIPPER,
	STATE_MOVE_GRIPPER,
	STATE_ARM_REVERSE,
	STATE_BACK_TO_LINE,
	STATE_INCH_Y,
	STATE_SET_TASK,
	STATE_PICK_STORAGE,
	STATE_PICK_SUPPLY,
} state;

// Field radiation level
enum radiation_t {
	RAD_HIGH = 3,
	RAD_LOW = 2,
	RAD_NONE = 1,
} radiation;

//**************************************************************/
// LIMIT SWITCHES
//**************************************************************/

const uint8_t PIN_SWITCH_REACTOR = 25;
const uint8_t PIN_SWITCH_TUBE = 24;

LimitSwitch reactorSwitch(PIN_SWITCH_REACTOR);
LimitSwitch tubeSwitch(PIN_SWITCH_TUBE);

//**************************************************************/
// HELPER FUNCTION DEFINITIONS
//**************************************************************/

// Returns true if robot is at either reactor
bool atReactor() {
	return (currentPos == REACTOR_A)
		|| (currentPos == REACTOR_B);
}

// Resets both drive encoders then transitions to given state.
void resetEncoders(state_t nextState) {
	MotorL::motor.zeroAngle();
	MotorR::motor.zeroAngle();
	state = nextState;
}

// Inches forward by fixed angle then transitions to given state.
void inchForward(state_t nextState) {
	LineFollower::drive();
	if((MotorL::motor.getAngle() +
		MotorR::motor.getAngle()) >= 2.0 * VTC_INCH_ANGLE)
	{
		state = nextState;
	}
}

//**************************************************************/
// MAIN FUNCTION DEFINITIONS
//**************************************************************/

// Initializes ReactorBot (call in setup).
void robotSetup() {

	// Namespace initializations
	MotorL::setup();
	MotorR::setup();
	Arm::setup();
	Gripper::setup();
	GyroDrive::setup();
	LineFollower::setup();
	Bluetooth::setup();
	IndicatorLed::setup();

	// Limit Switch initializations
	reactorSwitch.setup();
	tubeSwitch.setup();

	// Open gripper and raise arm to back position
	Gripper::open();
	while(!Gripper::ready());
	while(!Arm::setAngle(Arm::ANGLE_BACK));

	// State machine initialization
	state = STATE_BEGIN;
}

// Robot state machine loop (call in loop).
void robotLoop() {

	// Bluetooth communication
	Bluetooth::loop(radiation);

	// State Machine
	switch(state) {

		// Initialize state machine
		case STATE_BEGIN:
			reactor = A;
			task = TASK_EMPTY_REACTOR;
			state = STATE_DECIDE_X;
			radiation = RAD_NONE;
			targetPos = REACTOR_A;
			break;

		// Decide on x turning direction
		case STATE_DECIDE_X:
			if(targetPos.x == currentPos.x)
				state = STATE_DECIDE_Y;
			else {
				if(targetPos.x > currentPos.x)
					targetHeading = HEADING_R;
				else
					targetHeading = HEADING_L;
				state = STATE_TURNTO_X;
			}
			break;

		// Gyro turn to face target position x
		case STATE_TURNTO_X:
			if(GyroDrive::setAngle(targetHeading))
				state = STATE_GOTO_X;
			break;

		// Line follow to target position x
		case STATE_GOTO_X:
			LineFollower::drive();
			if(LineFollower::hitIntersection()) {
				if(GyroDrive::heading() < PI) currentPos.x++;
				else currentPos.x--;
			}
			if(currentPos.x == targetPos.x) {
				switch(task) {
					case TASK_EMPTY_REACTOR:
						state = STATE_APPROACH_REACTOR;
						break;
					case TASK_FILL_REACTOR:
						MotorL::motor.brake();
						MotorR::motor.brake();
						state = STATE_PREP_DEPOSIT_1;
						break;
					default:
						resetEncoders(STATE_INCH_X);
						break;
				}
			}
			break;

		// Lower arm halfway to deposit reactor rod
		case STATE_PREP_DEPOSIT_1:
			if(Arm::setAngle(Arm::ANGLE_PREP_1))
				state = STATE_PREP_DEPOSIT_2;
			break;

		// Raise arm up a bit to avoid reactor collision
		case STATE_PREP_DEPOSIT_2:
			if(Arm::setAngle(Arm::ANGLE_PREP_2))
				state = STATE_APPROACH_REACTOR;
			break;

		// Line follow until reactor limit switch contact
		case STATE_APPROACH_REACTOR:
			LineFollower::drive(2.0);
			if(reactorSwitch.pressed())
				state = STATE_DECIDE_ARM;
			break;

		// Inch forward until robot VTC is on line intersection
		case STATE_INCH_X:
			inchForward(STATE_DECIDE_Y);
			break;

		// Decide on y turning direction
		case STATE_DECIDE_Y:
			if(targetPos.y == currentPos.y)
				state = STATE_DECIDE_ARM;
			else {
				if(targetPos.y > currentPos.y)
					targetHeading = HEADING_U;
				else
					targetHeading = HEADING_D;
				state = STATE_TURNTO_Y;
			}
			break;

		// Turn to face target position y
		case STATE_TURNTO_Y:
			if(GyroDrive::setAngle(targetHeading))
				state = STATE_GOTO_Y;
			break;

		// Line follow until tube limit switch contact
		case STATE_GOTO_Y:
			LineFollower::drive();
			if(tubeSwitch.pressed()) {
				switch(task) {
					case TASK_FILL_STORAGE:
						currentPos.y = +1;
						break;
					case TASK_GET_SUPPLY:
						currentPos.y = -1;
						break;
					default: break;
				}
				state = STATE_DECIDE_ARM;
			}
			break;

		// Stop driving and choose arm forward position
		case STATE_DECIDE_ARM:
			MotorL::motor.brake();
			MotorR::motor.brake();
			switch(task) {
				case TASK_EMPTY_REACTOR:
					targetArmAngle = Arm::ANGLE_PICKUP;
					break;
				case TASK_FILL_REACTOR:
					targetArmAngle = Arm::ANGLE_DROPOFF;
					break;
				default:
					targetArmAngle = Arm::ANGLE_TUBE;
			}
			state = STATE_ARM_FORWARD;
			break;

		// Move arm to forward position
		case STATE_ARM_FORWARD:
			if(Arm::setAngle(targetArmAngle))
				state = STATE_DECIDE_GRIPPER;
			break;

		// Choose gripper action
		case STATE_DECIDE_GRIPPER:
			switch(task) {
				case TASK_EMPTY_REACTOR:
				case TASK_GET_SUPPLY:
					Gripper::close();
					break;
				default:
					Gripper::open();
					break;
			}
			state = STATE_MOVE_GRIPPER;
			break;

		// Wait for gripper to finish action
		case STATE_MOVE_GRIPPER:
			if(Gripper::ready()) {
				switch(task) {
					case TASK_GET_SUPPLY:
						radiation = RAD_HIGH;
						break;
					case TASK_EMPTY_REACTOR:
						radiation = RAD_LOW;
						break;
					default:
						radiation = RAD_NONE;
						break;
				}
				state = STATE_ARM_REVERSE;
			}
			break;

		// Move arm to back position
		case STATE_ARM_REVERSE:
			if(Arm::setAngle(Arm::ANGLE_BACK))
				state = STATE_BACK_TO_LINE;
			break;

		// Gyro drive backwards to line intersection
		case STATE_BACK_TO_LINE:
			GyroDrive::setVelocity(0.0, -4.0); // Drive backwards
			if(LineFollower::hitIntersection()) {
				currentPos.y = 0;
				if(atReactor()) state = STATE_SET_TASK;
				else resetEncoders(STATE_INCH_Y);
			}
			break;

		// Inch forward until robot VTC is on line intersection
		case STATE_INCH_Y:
			inchForward(STATE_SET_TASK);
			break;

		// Set next robot task, state, and target position
		case STATE_SET_TASK:
			MotorL::motor.brake();
			MotorR::motor.brake();
			switch(task) {
				case TASK_EMPTY_REACTOR:
					task = TASK_FILL_STORAGE;
					state = STATE_PICK_STORAGE;
					break;
				case TASK_FILL_STORAGE:
					task = TASK_GET_SUPPLY;
					state = STATE_PICK_SUPPLY;
					break;
				case TASK_GET_SUPPLY:
					task = TASK_FILL_REACTOR;
					switch(reactor) {
						case A: targetPos = REACTOR_A; break;
						case B: targetPos = REACTOR_B; break;
					}
					state = STATE_DECIDE_X;
					break;
				case TASK_FILL_REACTOR:
					task = TASK_EMPTY_REACTOR;
					switch(reactor) {
						case A:
							reactor = B;
							targetPos = REACTOR_B;
							break;
						case B:
							reactor = A;
							targetPos = REACTOR_A;
							break;
					}
					state = STATE_DECIDE_X;
					break;
			}
			break;

		// Set target position to closest available storage tube
		case STATE_PICK_STORAGE:
			switch(reactor) {
				case A: // Storage 4 is closest
					for(int i=4; i>=1; i--)
						if(Bluetooth::com.storageAvailable(i)) {
							targetPos = STORAGE[i-1];
							state = STATE_DECIDE_X;
							break;
						}
					break;
				case B: // Storage 1 is closest
					for(int i=1; i<=4; i++)
						if(Bluetooth::com.storageAvailable(i)) {
							targetPos = STORAGE[i-1];
							state = STATE_DECIDE_X;
							break;
						}
					break;
			}
			break;

		// Set target position to closest available supply tube
		case STATE_PICK_SUPPLY:
			switch(reactor) {
				case A: // Supply 1 is closest
					for(int i=1; i<=4; i++)
						if(Bluetooth::com.supplyAvailable(i)) {
							targetPos = SUPPLY[i-1];
							state = STATE_DECIDE_X;
							break;
						}
					break;
				case B: // Supply 4 is closest
					for(int i=4; i>=1; i--)
						if(Bluetooth::com.supplyAvailable(i)) {
							targetPos = SUPPLY[i-1];
							state = STATE_DECIDE_X;
							break;
						}
					break;
			}
			break;
	}

	// Update radiation indicator LED
	switch(radiation) {
		case RAD_HIGH: IndicatorLed::setHigh(); break;
		case RAD_LOW:  IndicatorLed::setLow();  break;
		case RAD_NONE: IndicatorLed::setNone(); break;
	}
}
