
#pragma once

#include "config/stm32plus.h"
#include "board/main_v3.h"

#include "control/controlTimer.h"
#include "utils/MadgwickAHRS.h"

#include "machine/kalman.h"

#ifndef M_PI
#define M_PI 3.1415926535
#endif
#define GYRO_P 15 //base 17
#define GYRO_D 0

void setLimit(int &value,int limit);

enum{
	TURN_LEFT  = -1,
	TURN_RIGHT =  1,
	MODE_X     =  0,
	MODE_Y         ,
	X          =  0,
	Y
};

class Machine {
private:
	int anti_slip[2]  = {},
	    auto_runXY[2] = {},
		enc[2]        = {},
		enc_now[2]    = {},
		enc_old[2]    = {},
		enc_diff[2]   = {},
		target_loc[2] = {},
		XY_100        = 0;

public:

	int antiSlip(int XY_100, int mode) {

		if (XY_100 == 0) {
			anti_slip[mode] = 0;
		} else {
			anti_slip[mode] += (XY_100 - anti_slip[mode]) / 5;
		}

		return anti_slip[mode];
	}

	int runToTarget(int mode) {
		int target_distance = 0;

		target_distance = enc[mode] - target_loc[mode];

		if (target_distance >= 10 || target_distance <= -10) {
			if (checkMove(mode) == false) {
				if      (auto_runXY[mode] < 0) auto_runXY[mode] -= 35;
				else if (auto_runXY[mode] > 0) auto_runXY[mode] += 35;
			}
				auto_runXY[mode] += ((target_distance / 15) - auto_runXY[mode]) / 3;
		} else {
			auto_runXY[mode] = 0;
		}

		setLimit(auto_runXY[mode], 250);

		return auto_runXY[mode];
	}

	void setTargetLocation(){
		target_loc[X] = enc[X];
		target_loc[Y] = enc[Y];
	}

	//Automatically adjust the distance between the object
	int adjustDistance(int distance,int target,int mode){

		if (distance > target + 25 || distance < target - 25) {
			XY_100 = (distance - target) / 3;

			if(checkMove(mode) == false){
				if(XY_100 > 0)      XY_100 += 20;
				else if(XY_100 < 0) XY_100 -= 20;
			}
		}else{
			XY_100 = 0;
		}

		setLimit(XY_100, 100);

		return XY_100;
	}

	void extendEncValue(int now_value,int old_value,int mode){
		int _enc = 0;

		if (now_value - old_value < 20000 && now_value - old_value > -20000) {
			_enc = now_value - old_value;
		}else if(now_value < 1000){
			_enc = now_value;
		}

		enc_now[mode] = now_value;
		enc_old[mode] = old_value;
		enc_diff[mode] = _enc;

		enc[mode] += _enc;
	}

	bool checkMove(int mode){
		bool check = false;

		if(abs(enc_diff[mode]) > 10) check = true;

		return check;
	}

};

class Gyro{
private:
	int   count_180     = 0;
	float yaw_90value   = 0,
		  yaw_90old     = 0,
		  yaw_value     = 0,
		  yaw_value_old = 0,
		  yaw_reset     = 0;

	bool first_time = true;

	Machine machine;

public:
	void set(float yaw_now,float yaw_old){

		if (yaw_now - yaw_old > 0.04 || yaw_now - yaw_old < -0.04){
			yaw_value_old = yaw_value;

			if(abs(yaw_now + yaw_old) < 20 && (yaw_now < -150 || yaw_now > 150)){

				if(yaw_old > 150)       count_180++;
				else if(yaw_old < -150) count_180--;

				yaw_value += 180 * count_180 + yaw_now + (180 * count_180 - yaw_value);
			}else{
				yaw_value += yaw_now - yaw_old;
			}
		}

	}

	int angle90(int mode){
		int rotation_90 = 0;

		if(first_time == true){
			yaw_90old = yaw_value;
			first_time = false;
		}

		if(abs(yaw_value - yaw_90old) < 20) yaw_90value += abs(yaw_value - yaw_90old);

		if(yaw_90value < 90) rotation_90 = 120 - yaw_90value;
		else                 rotation_90 = 0;

		yaw_90old = yaw_value;

		if(mode == TURN_LEFT) rotation_90 *= -1;

		return rotation_90;
	}
	void resetAngle90(){
		first_time = true;
		yaw_90value = 0;
	}

	int correct(){
		int rotation_  = 0,
			rotation_P = 0,
			rotation_D = 0;

		rotation_P = (yaw_value - yaw_reset) * GYRO_P;
		rotation_D = (yaw_value - yaw_value_old) * GYRO_D;

		rotation_ = rotation_P + rotation_D;

		if (machine.checkMove(MODE_X) == false){
			if      (rotation_ > 0) rotation_ += 2;
			else if (rotation_ < 0) rotation_ -= 2;
		}

		return rotation_;
	}

	void reset(int &rotation){
		yaw_reset = yaw_value;
		rotation = 0;
	}
};

class Build{
private:
	int b_mode = -1;

	MainV3 mainBoard;

public:
	void changeMode(int get_mode = 99){
		if(get_mode == 99) b_mode++;

		switch(b_mode){
		case 0:
		case 4:
		case 8:
			mainBoard.servoA.On();//Arm open.
			mainBoard.servoB.Off();
			mainBoard.servoC.Off();
			break;
		case 1:
		case 5:
		case 9:
			mainBoard.servoA.On();
			mainBoard.servoB.On();//Get objetc
			mainBoard.servoC.Off();
			break;
		case 2:
		case 6:
		case 10:
			mainBoard.servoA.Off();//Arm close
			mainBoard.servoB.On();
			mainBoard.servoC.Off();
			break;
		case 3:
		case 7:
		case 11:
			mainBoard.servoA.Off();
			mainBoard.servoB.Off();//Release object
			mainBoard.servoC.Off();
			break;
		case 12:
			mainBoard.servoA.Off();
			mainBoard.servoB.Off();
			mainBoard.servoC.On();//Push capital
			break;
		default:
			b_mode = -1;
			mainBoard.servoA.Off();
			mainBoard.servoB.Off();
			mainBoard.servoC.Off();
			break;
		}
	}

	int getMode(){
		return b_mode;
	}

	void Reset(){
		b_mode = -1;
		changeMode(-1);
	}
};


class SensorTimer {
public:
	MainV3 *mainBoard;
	MadgwickAHRS ahrs;

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

void setLimit(int &value,int limit){
	value = min(max(value, - limit), limit);
}
