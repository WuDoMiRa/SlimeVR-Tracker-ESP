/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2022 TheDevMinerTV

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#include "LEDManager.h"

#include "GlobalVars.h"

namespace SlimeVR {
void LEDManager::setup() {
#if ENABLE_LEDS
	pinMode(m_Pin, OUTPUT);
#endif

	// Do the initial pull of the state
	update();
}

void LEDManager::on() {
#if ENABLE_LEDS
	digitalWrite(m_Pin, LED__ON);
#endif
}

void LEDManager::off() {
#if ENABLE_LEDS
	digitalWrite(m_Pin, LED__OFF);
#endif
}

void LEDManager::blink(unsigned long time) {
	on();
	delay(time);
	off();
}

void LEDManager::pattern(unsigned long timeon, unsigned long timeoff, int times) {
	for (int i = 0; i < times; i++) {
		blink(timeon);
		delay(timeoff);
	}
}
}