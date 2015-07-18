
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
	    auto_runXY[2] = {0},
		XY_100         = 0;

	float yaw_90value = 0,
		  yaw_90old   = 0;

	bool first_time = true;

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

	int antiSlip(int XY_100, int i) {

		if (XY_100 == 0) {
			anti_slip[i] = XY_100;
		} else {
			anti_slip[i] += (XY_100 - anti_slip[i]) / 5;
		}

		return anti_slip[i];
	}

	int Auto_Runner(int t_e, int x_or_y, int enc_old, int enc_now) {
		if (t_e >= 10 || t_e <= -10) {
			if (abs(enc_old - enc_now) < 10) {

				if      (auto_runXY[x_or_y] < 0) auto_runXY[x_or_y] -= 30;
				else if (auto_runXY[x_or_y] > 0) auto_runXY[x_or_y] += 30;

			}
				auto_runXY[x_or_y] += ((t_e / 15) - auto_runXY[x_or_y]) / 3;

		} else {
			auto_runXY[x_or_y] = 0;
		}

		setLimit(auto_runXY[x_or_y], 250);

		return auto_runXY[x_or_y];
	}

	int rotateAngle90(float yaw_value,int mode){
		int rotation_90 = 0;

		if(first_time == true){
			yaw_90old = yaw_value;
			first_time = false;
		}

		if(abs(yaw_value - yaw_90old) < 20){
			if(mode == 1) yaw_90value += abs(yaw_value - yaw_90old);
			else if(mode == 2) yaw_90value -= abs(yaw_value - yaw_90old);
		}

		if(yaw_90value < 90) rotation_90 = 120 - yaw_90value;
		else {
			rotation_90 = 0;
			//mainBoard.buzzer.set(-1, 6);
		}

		yaw_90old = yaw_value;

		return rotation_90;
	}

	//Automatically adjust the distance between the object
	int autoAdjustDistance(int distance,int target,int enc_now,int enc_old){

		if (distance > target + 25 || distance < target - 25) {
			XY_100 = (distance - target) / 3;

			if(abs(enc_now - enc_old) < 10){
				if(XY_100 > 0)      XY_100 += 30;
				else if(XY_100 < 0) XY_100 -= 30;
			}
		}else{
			XY_100 = 0;
		}

		setLimit(XY_100, 100);

		return XY_100;
	}

	void resetRotateAngle90(){
		first_time = true;
		yaw_90value = 0;
	}

	void setLimit(int &value,int limit){
		if (value > limit)  value = limit;
		else if (value < -limit) value = -limit;
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
