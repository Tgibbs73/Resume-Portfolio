//**************************************************************/
// TITLE
//**************************************************************/

// FieldPosition.h
// Class for keeping field positions for ReactorBot.
// RBE-2001 A17 Team 7

#pragma once

//**************************************************************/
// CLASS DECLARATION
//**************************************************************/

class FieldPosition {
public:
	FieldPosition(int x, int y) {
		this->x = x;
		this->y = y;
	}
	bool operator==(const FieldPosition& fp) {
		return ((x == fp.x) && (y == fp.y));
	}
	int x = 0;
	int y = 0;
};

//**************************************************************/
// OBJECT DECLARATIONS
//**************************************************************/

// Reactors
const FieldPosition REACTOR_A(+1, +0);
const FieldPosition REACTOR_B(+6, +0);

// Storage containers
const FieldPosition STORAGE[4]{
	FieldPosition(+5, +1),
	FieldPosition(+4, +1),
	FieldPosition(+3, +1),
	FieldPosition(+2, +1),
};

// Supply containers
const FieldPosition SUPPLY[4]{
	FieldPosition(+2, -1),
	FieldPosition(+3, -1),
	FieldPosition(+4, -1),
	FieldPosition(+5, -1),
};
