//**************************************************************/
// TITLE
//**************************************************************/

// ReactorBot.ino
// Executed code for ReactorBot RBE project.
// RBE-2001 A17 Team 7

#include "StateMachine.h"

//**************************************************************/
// MAIN FUNCTION DEFINITIONS
//**************************************************************/

// Runs once on Arduino reset.
void setup() {
	robotSetup();
}

// Runs repeatedly after setup.
void loop() {
	robotLoop();
}
