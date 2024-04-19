//**************************************************************/
// TITLE
//**************************************************************/

//!t ReactorComms.h
//!b Class for RBE-2001 final project Bluetooth communication.
//!a Dan Oates (WPI Class of 2020)

//!d This class utilizes the HC-05 Bluetooth serial bridge
//!d module through any hardware serial port available on
//!d any Arduino. The serial port is specified on object
//!d construction.

#pragma once
#include "Arduino.h"

//**************************************************************/
// CONSTANT DEFINITIONS
//**************************************************************/

const bool RADIATION_HI = true;
const bool RADIATION_LO = false;

//**************************************************************/
// CLASS DECLARATION
//**************************************************************/

class ReactorComms {
public:
	ReactorComms(HardwareSerial& serial);
	void init();

	void update();
	bool getRobotEnabled();
	bool storageAvailable(int);
	bool supplyAvailable(int);

	void sendHeartBeat();
	void sendRadAlert(bool);
private:
	HardwareSerial* serial;

	bool robotEnabled = false;
	byte storData = 0x00;
	byte fuelData = 0x00;

	byte checkSum = 0xFF;
	byte read();
	void write(byte);
};
