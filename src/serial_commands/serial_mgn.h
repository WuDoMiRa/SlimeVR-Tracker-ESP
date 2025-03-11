#include "../GlobalVars.h"
#include "../globals.h"
#include "cmds/wifi.h"
SlimeVR::WifiSerialCommand WifiCMD;
namespace SlimeVR {
    /// @brief Reads from serial, into a complete buffer. And executes commands accordingly.
    void ReadSerial() {
        SlimeVR::Logger logger(Serial, "SlimeVR", "Serial Commands"); // the logger.
        static char buffer[256];
        static int buffer_index = 0;
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n') {
                buffer[buffer_index] = '\0';
                buffer_index = 0;

                // Convert buffer to string
                String complete_string(buffer);

                // extract prefix
                String prefix = complete_string.substring(0, complete_string.indexOf(' '));
                String lowercaseprefix = String(prefix); // copy
                lowercaseprefix.toLowerCase();

                // extract arguments from the string into a vector. Keeping spaces that are in quotes (' or ")
                std::vector<String> arguments;
                int start = complete_string.indexOf(' ');
                int end = complete_string.length();
                while (start != -1) {
                    while (complete_string[start] == ' ') {
                        start++;
                    }
                    if (complete_string[start] == '\"' || complete_string[start] == '\'') {
                        start++;
                        end = complete_string.indexOf(complete_string[start], start);
                        arguments.push_back(complete_string.substring(start, end));
                        start = complete_string.indexOf(' ', end);
                    } else {
                        end = complete_string.indexOf(' ', start);
                        arguments.push_back(complete_string.substring(start, end));
                        start = complete_string.indexOf(' ', end);
                    }
                }

                
                
                // I've been thinking about some implementations for this, all the possible implementations I thought of include:
                // some form of declaring a map of functions, which still requires including all the commands,
                // even a switch still acts the same of a map but still enclosed,
                // you cannot have a list of commands where if you loop through and the prefix matches you execute it because,
                // making if checks would be more efficient than storing commands into a list, and even then you'd still have to insert the commands into the list, 
                // you are allocating a vector everytime you run the function, and also creating structs, thus
                // you cannot exactly have a 'dynamic' implementation unfortunately in C, C++.

                // Make a switch that checks the prefix, and then runs the command.
                if (lowercaseprefix == WifiCMD.name) {
                    return WifiCMD.run(arguments);
                }
                logger.print("Unknown command: %s", prefix.c_str());
            } else {
                buffer[buffer_index] = c;
                buffer_index++;
            }
        }
    }
}