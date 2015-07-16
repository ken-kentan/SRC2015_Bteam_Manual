
#pragma once

#include "config/stm32plus.h"
#include "board/main_v3.h"

#include "control/controlTimer.h"
#include "utils/MadgwickAHRS.h"

#ifndef M_PI
#define M_PI 3.1415926535
#endif

class Machine {
private:
	int anti_slip[2]  = {0},
	    auto_runXY[2] = {0};

public:

	int motorDriver_Protecter(int IN_Mout) {
		int ret;

		if (IN_Mout > 499) {
			ret = 499;
		} else if (IN_Mout < -499) {
			ret = -499;
		} else {
			ret = IN_Mout;
		}
		return ret;
	}

	int Anti_sliper(int XY_100, int i) {

		if (XY_100 == 0) {
			anti_slip[i] = XY_100;
		} else {
			anti_slip[i] += (XY_100 - anti_slip[i]) / 5;
		}

		return anti_slip[i];
	}

	int Auto_Runner(int t_e, int x_or_y, int enc_old, int enc_now) {
		if (t_e >= 50 || t_e <= -50) {
			if (enc_old - enc_now == 0) {

				if      (auto_runXY[x_or_y] < 0) auto_runXY[x_or_y] -= 5;
				else if (auto_runXY[x_or_y] > 0) auto_runXY[x_or_y] += 5;

			} else {
				auto_runXY[x_or_y] += ((t_e / 15) - auto_runXY[x_or_y]) / 3;
			}
		} else {
			auto_runXY[x_or_y] = 0;
		}

		if      (auto_runXY[x_or_y] >=  250) auto_runXY[x_or_y] =  250;
		else if (auto_runXY[x_or_y] <= -250) auto_runXY[x_or_y] = -250;

		return auto_runXY[x_or_y];
	}

	void set_limit(int &value,int limit){
		if (value > limit)  value = limit;
		if (value < -limit) value = -limit;
	}
};


class SensorTimer {
public:
	MainV3 *mainBoard;
	MadgwickAHRS ahrs;

	float yaw=0, pitch, roll;
	float sum = 0;

	int16_t acc[3] = { 0 };
	int16_t gyr[3] = { 0 };
	float angular_velocity[3], linear_acceleration[3];

	SensorTimer(MainV3 *_mainBoard) :
			ahrs() {
		mainBoard = _mainBoard;

		timer.setTimeBaseByFrequency(10000, 100);
		// 割り込みハンドラをバインド
		timer.TimerInterruptEventSender.insertSubscriber(
				TimerInterruptEventSourceSlot::bind(this,
						&SensorTimer::onInterrupt));
		timer.setNvicPriorities(1, 0);
		timer.enableInterrupts(TIM_IT_Update); // オーバーフロー割り込み有効化
		timer.enablePeripheral();
	}

	// 割り込みハンドラ
	void onInterrupt(TimerEventType tet, uint8_t timerNumber) {
		// オーバーフロー割り込みの場合
		if (tet == TimerEventType::EVENT_UPDATE) {

			mainBoard->mpu6050.readAccAll(acc);
			mainBoard->mpu6050.readGyrAll(gyr);

			angular_velocity[0] = (float) gyr[0] / 32768.0f * 500.0f
					* (M_PI / 180.0f);
			angular_velocity[1] = (float) gyr[1] / 32768.0f * 500.0f
					* (M_PI / 180.0f);
			angular_velocity[2] = (float) gyr[2] / 32768.0f * 500.0f
					* (M_PI / 180.0f);

			yaw += angular_velocity[2];

			linear_acceleration[0] = (float) acc[0] / 32768.0f * 8.0f;
			linear_acceleration[1] = (float) acc[1] / 32768.0f * 8.0f;
			linear_acceleration[2] = (float) acc[2] / 32768.0f * 8.0f;

			ahrs.update(0.01, angular_velocity[0], angular_velocity[1],
					angular_velocity[2], linear_acceleration[0],
					linear_acceleration[1], linear_acceleration[2]);

		}
	}

private:
	int count = 0;
	Timer6<Timer6InternalClockFeature, Timer6InterruptFeature> timer;
};
