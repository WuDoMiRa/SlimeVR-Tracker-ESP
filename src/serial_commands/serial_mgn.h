#ifndef SERIAL_MANAGER_H
#define SERIAL_MANAGER_H

#include "globals.h"
namespace SlimeVR {
    /// @brief Reads from serial, into a complete buffer. And executes commands accordingly.
	struct SerialManager {
		/// @brief reads from serial and executes commands accordingly
		void ReadSerial();
		/// @brief sets up commands that require a manual setup. i.e wifi
		void SetupCMDs();
	};
}

#endif
