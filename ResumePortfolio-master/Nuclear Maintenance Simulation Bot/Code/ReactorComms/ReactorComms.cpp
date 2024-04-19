//**************************************************************/
// TITLE
//**************************************************************/

//!t ReactorComms.cpp
//!a Dan Oates (Team 7, Class of 2020)

#include "ReactorComms.h"

//**************************************************************/
// CONSTRUCTOR DEFINITIONS
//**************************************************************/

//!b Constructs ReactorComms through given hardware serial port.
ReactorComms::ReactorComms(HardwareSerial& serial) {
	this->serial = &serial;
}

//**************************************************************/
// PUBLIC METHOD DEFINITIONS
//**************************************************************/

//!b Initializes serial communication with the HC-05.
void ReactorComms::init() {
	serial->begin(115200);
}

//!b Processes new message packets from reactor control module.
//!d Ignores packets if:
//!d - They are not from the reactor control module
//!d - They are intended for another robot
//!d - The have incorrect checksums
void ReactorComms::update() {
	while(serial->available()) {
		if(serial->read() == 0x5F) { // Search for start

			// Read full message
			checkSum = 0xFF;
			byte len = read() + 1;
			byte msg[len];
			msg[0] = 0x5F;
			msg[1] = len;
			for(int i=2; i<len; i++) msg[i] = read();

			// Check read conditions
			if(checkSum == 0x00 // Checksum passed
			  && msg[3] == 0x00 // Source is field
			  //&& msg[4] == 0x07 // Destination is Team 7
			){
				// Check message type
				switch(msg[2]) {
					case 0x01: // Storage tube availability
						storData = msg[5];
						break;
					case 0x02: // Supply tube availability
						fuelData = msg[5];
						break;
					case 0x04: // Stop movement
						robotEnabled = false;
						break;
					case 0x05: // Resume movement
						robotEnabled = true;
						break;
				}
			}
		}
	}
}

//!b Returns true if the robot is enabled by reactor control.
bool ReactorComms::getRobotEnabled() {
	return robotEnabled;
}

//!b Returns true if given storage tube is empty.
//!i Storage tube ID (valid 1-4).
bool ReactorComms::storageAvailable(int id) {
	switch(id) {
		case 1: return (storData & B00000001) != B00000001;
		case 2: return (storData & B00000010) != B00000010;
		case 3: return (storData & B00000100) != B00000100;
		case 4: return (storData & B00001000) != B00001000;
		default: return false;
	}
}

//!b Returns true if given supply tube is full.
//!i Supply tube ID (valid 1-4)
bool ReactorComms::supplyAvailable(int id) {
	switch(id) {
		case 1: return (fuelData & B00000001) == B00000001;
		case 2: return (fuelData & B00000010) == B00000010;
		case 3: return (fuelData & B00000100) == B00000100;
		case 4: return (fuelData & B00001000) == B00001000;
		default: return false;
	}
}

//!b Sends one heart-beat message to reactor control.
void ReactorComms::sendHeartBeat() {
	checkSum = 0xFF;
	write(0x5F); // Start delimeter
	write(0x05); // 5-byte message
	write(0x07); // Heart beat type
	write(0x07); // From robot 7
	write(0x00); // To reactor control
	write(checkSum);
}

//!b Sends radiation alert to reactor control.
//!i True for new rod, false for spent rod
void ReactorComms::sendRadAlert(bool high) {
	checkSum = 0xFF;
	write(0x5F); // Start delimeter
	write(0x06); // 6-byte message
	write(0x03); // Radiation alert type
	write(0x07); // From robot 7
	write(0x00); // To reactor control
	if(high)
		write(0xFF); // New fuel rod
	else
		write(0x2C); // Spent fuel rod
	write(checkSum);
}

//**************************************************************/
// PRIVATE METHOD DEFINITIONS
//**************************************************************/

//!d Reads 1 byte from the serial buffer and
//!d decrements it from the checksum.
byte ReactorComms::read() {
	while(!serial->available());
	byte b = serial->read();
	checkSum -= b;
	return b;
}

//!d Writes 1 byte to the serial buffer and
//!d decrements it from the checksum.
void ReactorComms::write(byte b) {
	serial->write(b);
	checkSum -= b;
}
