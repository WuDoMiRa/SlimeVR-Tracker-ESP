#ifndef SERIAL_MANAGER_H
#define SERIAL_MANAGER_H

#include "globals.h"
namespace SlimeVR {
    /// @brief Reads from serial, into a complete buffer. And executes commands accordingly.
	struct SerialManager {
		void ReadSerial();
	};
}

#endif
