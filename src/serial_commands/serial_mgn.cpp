#include "GlobalVars.h"
#include "serial_mgn.h"
#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#if !ESP8266
#include "esp_wifi.h"
#endif
#include "cmds/wifi.h"
#include <vector>
/// TODO: in the future:
// 1. make a base command class/struct
// 2. make a vector variable here, that will contain all commands
// 3. when checking if a command is called, iterate through the vector to
// 		execute the commands dynamically. you still have to include them though.
SlimeVR::WifiSerialCommand WifiCMD;
namespace SlimeVR {
	void SerialManager::ReadSerial() {
		SlimeVR::Logger logger(Serial, "SlimeVR", "Serial Commands"); // the logger.
		static char buffer[256];
		static int buffer_index = 0;
		while (Serial.available() > 0) {
			char c = Serial.read();
			/// added a check below so that we dont have a buffer overflow when reading from serial, potentially.
			if ((c == '\n') || (buffer_index==256)) {
				buffer[buffer_index] = '\0';
				buffer_index = 0;

				// Convert buffer to string
				String complete_string(buffer);
				// Remove any special characters from the string
				complete_string.replace('\r', ' ');
				complete_string.replace('\n', ' ');
				complete_string.replace('\t', ' ');
				complete_string.replace('\v', ' ');
				complete_string.replace('\f', ' ');

				// extract prefix
				String prefix = complete_string.substring(0, complete_string.indexOf(' '));
				String lowercaseprefix = String(prefix); // copy
				lowercaseprefix.toLowerCase();

				// extract arguments from the string into a vector. Keeping spaces that are in quotes (' or ")
				std::vector<String> arguments;
				bool in_quotes = false;
				char quote_char = '\0';
				String current_arg;

				for (size_t i = prefix.length() + 1; i < complete_string.length(); i++) {
					char c = complete_string[i];
					
					if ((c == '\'' || c == '"') && (!in_quotes || c == quote_char)) {
						in_quotes = !in_quotes;
						quote_char = in_quotes ? c : '\0';
						continue;
					}
					
					if (c == ' ' && !in_quotes) {
						if (!current_arg.isEmpty()) {
							arguments.push_back(current_arg);
							current_arg = "";
						}
					} else {
						current_arg += c;
					}
				}

				if (!current_arg.isEmpty()) {
					arguments.push_back(current_arg);
				}



				// I've been thinking about some implementations for this, all the possible implementations I thought of include:
				// some form of declaring a map of functions, which still requires including all the commands,
				// even a switch still acts the same of a map but still enclosed,
				// you cannot have a list of commands where if you loop through and the prefix matches you execute it because,
				// making if checks would be more efficient than storing commands into a list, and even then you'd still have to insert the commands into the list,
				// you are allocating a vector everytime you run the function, and also creating structs, thus
				// you cannot exactly have a 'dynamic' implementation unfortunately in C, C++.

				// Make a switch that checks the prefix, and then runs the command.

				/// TODO: There's an issue with serial commands where:
				// text could appear to be 'wifi' on the terminal, but the check below fails and just defaults to unknown command, which means the check is failing.
				// might be hidden characters at play EDIT: it was. hidden characters are (\n,\r,\t, etc.)
				if (lowercaseprefix==WifiCMD.name) {
					return WifiCMD.run(arguments);
				}
				logger.print("Unknown command: %s", prefix.c_str());
				//delay(100);
				//logger.debug("What I received: %s, wifi command name: %s", lowercaseprefix, WifiCMD.name);
			} else {
				buffer[buffer_index] = c;
				buffer_index++;
			}
		}
	}
}
