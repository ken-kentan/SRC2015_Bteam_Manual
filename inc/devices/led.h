/*
 * led.h
 *
 *  Created on: 2014/10/05
 *      Author: spiralray
 *      customed:ken_kentan
 */

#pragma once

#include "config/stm32plus.h"
#include "config/gpio.h"

namespace stm32plus {

class Led {
private:
	GpioPinRef pin;
public:
	Led(GpioPinRef _pin) {
		pin = _pin;
	}

	void On() {
		pin.set();
	}
	void Off() {
		pin.reset();
	}

	void Flash() {
		On();
		MillisecondTimer::delay(100);
		Off();
		MillisecondTimer::delay(100);
	}
};

class Servo {
private:
	GpioPinRef pin;
public:
	Servo(GpioPinRef _pin) {
		pin = _pin;
	}

	void On() {
		pin.set();
	}
	void Off() {
		pin.reset();
	}
};

}
