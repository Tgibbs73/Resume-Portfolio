//**************************************************************/
// TITLE
//**************************************************************/

// IndicatorLed.h
// Namespace for ReactorBot indicator LED.
// RBE-2001 A17 Team 7

#pragma once
#include "Led.h"

//**************************************************************/
// NAMESPACE DEFINITION
//**************************************************************/

namespace IndicatorLed {

	// RGB LED
	const uint8_t PIN_R = 26;
	const uint8_t PIN_G = 27;
	const uint8_t PIN_B = 28;
	Led rLed(PIN_R);
	Led gLed(PIN_G);
	Led bLed(PIN_B);

	// Initializes LED (call in setup)
	void setup() {
		rLed.init();
		gLed.init();
		bLed.init();
	}

	// Sets LED to indicate high radiation.
	void setHigh() {
		rLed.on();
		gLed.off();
		bLed.off();
	}

	// Sets LED to indicate low radiation.
	void setLow() {
		rLed.off();
		gLed.on();
		bLed.off();
	}

	// Sets LED tp indicate no radiation.
	void setNone() {
		rLed.off();
		gLed.off();
		bLed.on();
	}
}
