#pragma once

#include "config/stm32plus.h"
#include "config/gpio.h"
#include "config/timer.h"

namespace stm32plus {
template<class TimBuzzer, class ChannelA, class ChannelB>
class Buzzer: TimBuzzer {
public:
	int MAX_COMPARE = 6;
	/*
	 enum {
	 MAX_COMPARE = 6
	 };
	 */
	Buzzer() :
			TimBuzzer() { //10000000
		this->setTimeBaseByFrequency(MAX_COMPARE, MAX_COMPARE - 1);
		ChannelA::initCompareForPwmOutput();
		ChannelB::initCompareForPwmOutput();
		this->enablePeripheral();
	}

	void set(float duty, int mode) {
		if (duty > 1.0f) {
			duty = 1.0f;
		} else if (duty < -1.0f) {
			duty = -1.0f;
		}
		duty = duty * 0.95;

		MAX_COMPARE = mode;
		this->setTimeBaseByFrequency(MAX_COMPARE, MAX_COMPARE - 1);

		if (MAX_COMPARE <= 1)
			MAX_COMPARE = 1;

		int16_t value = (int16_t) (duty * MAX_COMPARE);
		if (value > 0) {
			ChannelA::setCompare(MAX_COMPARE);
			ChannelB::setCompare(MAX_COMPARE - value);
		} else {
			ChannelA::setCompare(MAX_COMPARE + value);
			ChannelB::setCompare(MAX_COMPARE);
		}
	}
	inline void stop() {
		set(0, 6);
	}
};
}
