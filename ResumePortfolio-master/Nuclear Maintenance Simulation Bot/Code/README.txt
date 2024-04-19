INTRODUCTION

This folder contains all of the project-specific C-code used on the Arduino Mega 2650 microcontroller used as the brains of our robot. I also utilized many of my general-purpose Arduino libraries, which can be found at <https://github.com/doates625/ArduinoLibs>.

ORGANIZATION

This folder contains two sub-folders:

- ReactorBot: Contains all main robot code and namespaces
- ReactorComms: A class used to communicate with the field Bluetooth control module

NOTES

In all of the namespaces, functions are defined in the .h-file (no .cpp files were used). At the time of writing this code, I was new to namespaces and unfamiliar with the syntax and best practices for separating namespaces into .h and .cpp files. I did learn and implement these practices in my following project for RBE-2002.