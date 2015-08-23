
#pragma once

#include "config/stm32plus.h"
#include "board/main_v3.h"

#include "control/controlTimer.h"
#include "utils/MadgwickAHRS.h"


#ifndef M_PI
#define M_PI       3.1415926535
#endif

#define GYRO_P         10 //base 17
#define GYRO_D          2

#define HEIGHT_DEF   2600
#define HEIGHT_TOP   3667
#define HEIGHT_STAY  2200
#define HEIGHT_GET   1890
#define HEIGHT_OB1   2900
#define HEIGHT_OB2   3270
#define HEIGHT_OB3   3650

#define PLATE_TOP    1900
#define PLATE_BOTTOM 800

#define PS3_A_GAIN      3

#define NO_CAHNGE     999

void setLimit(int &value,int limit);
void setLimitFloat(float &value,float limit);


enum{
	TURN_LEFT  = -1,
	TURN_RIGHT =  1,
	X          =  0,
	Y              ,
	ARM        =  0,
	PLATE
};

class Machine {

private:
	int anti_slip[2]  = {},
	    auto_runXY[2] = {},
		target_loc[2] = {},
		XY_100        = 0;

	struct Encoder{
		int value[2] = {},
			now[2]   = {},
			old[2]   = {},
			diff[2]  = {};
	};

	Encoder enc;

public:
	int antiSlip(int XY_100,int mode) {
		int acc_rate = 5;

		if (XY_100 == 0) acc_rate = 3;
		else             acc_rate = 5;

		switch(acc_rate){
		case 0:
		case 4:
		case 8:
		case 10:
		case 11:
		case 12:
			acc_rate = 1;
			break;
		case 6:
		case 7:
			acc_rate = 2;
			break;
		}

		anti_slip[mode] += (XY_100 - anti_slip[mode]) / acc_rate;

		return anti_slip[mode];
	}

	int runToTarget(int mode) {
		int target_distance = 0;

		target_distance = enc.value[mode] - target_loc[mode];

		if (abs(target_distance) >= 10) {
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
		target_loc[X] = enc.value[X];
		target_loc[Y] = enc.value[Y];
	}

	//Automatically adjust the distance between the object
	int adjustDistance(int distance,int target,int mode){

		if (abs(distance - target) > 25) {
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

		enc.now[mode] = now_value;
		enc.old[mode] = old_value;
		enc.diff[mode] = _enc;

		enc.value[mode] += _enc;
	}

	bool checkMove(int mode){
		bool check = false;

		if(abs(enc.diff[mode]) > 10) check = true;

		return check;
	}
};

class Gyro {
private:
	int   count_180     = 0;

	bool first_time = true;

	struct Yaw{
		float value     = 0,
			  old       = 0,
			  offset    = 0,
			  value_90d = 0,
			  old_90d   = 0;
	};

	Yaw yaw;
	Machine *machine;

public:

	void set(float yaw_now,float yaw_old){

		if (abs(yaw_now - yaw_old) > 0.04){
			yaw.old = yaw.value;

			if(abs(yaw_now + yaw_old) < 20 && abs(yaw_now) > 150){

				if(yaw_old > 150)       count_180++;
				else if(yaw_old < -150) count_180--;

				yaw.value += 180 * count_180 + yaw_now + (180 * count_180 - yaw.value);
			}else{
				yaw.value += yaw_now - yaw_old;
			}
		}
	}

	int angle90(int mode){
		int rotation_90 = 0;

		if(first_time == true){
			yaw.old_90d = yaw.value;
			first_time = false;
		}

		if(abs(yaw.value - yaw.old_90d) < 20) yaw.value_90d += abs(yaw.value - yaw.old_90d);

		if(yaw.value_90d < 90) rotation_90 = 120 - yaw.value_90d;
		else                 rotation_90 = 0;

		yaw.old_90d = yaw.value;

		if(mode == TURN_LEFT) rotation_90 *= -1;

		return rotation_90;
	}

	void resetAngle90(){
		first_time = true;
		yaw.value_90d = 0;
	}

	int correct(){
		int rotation_  = 0,
			rotation_P = 0,
			rotation_D = 0;

		rotation_P = (yaw.value - yaw.offset) * GYRO_P;
		rotation_D = (yaw.value - yaw.old) * GYRO_D;

		rotation_ = rotation_P + rotation_D;

		if (machine->checkMove(X) == false){
			if      (rotation_ > 0) rotation_ += 0;
			else if (rotation_ < 0) rotation_ -= 0;
		}

		return rotation_;
	}

	void reset(int &rotation){
		yaw.offset = yaw.value;
		rotation = 0;
	}
};

class Builder{
private:
	int mode              = -2,
		potentiometer_old =  0,
		cnt_ture_arm      =  0,
		cnt_ture_plate    =  0,
		pwm_arm           =  0,
		pwm_plate         =  0,
		pause_time        =  0,
		pause_time_plate  =  0;

	bool completed_arm   = false,
		 completed_plate = false;

	MainV3 *mainBoard;
	Machine *machine;


public:
	Builder(MainV3 *mainv3,Machine *_machine){
		mainBoard = mainv3;
		machine = _machine;
	}

	void changeMode(int get_mode = 99){
		if(get_mode == 99){
			mode++;
		}else if(get_mode != NO_CAHNGE){
			mode = get_mode;
		}

		cnt_ture_arm++;
		cnt_ture_plate++;

		switch(mode){
		case -2:
		case -1:
			break;
		case 0:
		case 4:
		case 8:
			cnt_ture_arm = 0;
			mainBoard->servoA.Off();//Arm open.
			mainBoard->servoB.On();
			mainBoard->servoC.On();
			break;
		case 1:
		case 5:
		case 9:
			if(completed_arm == false || cnt_ture_arm < 5) break;
			cnt_ture_arm = 0;
			mainBoard->servoA.Off();
			mainBoard->servoB.Off();//Get objetc
			mainBoard->servoC.On();
			break;
		case 2:
		case 6:
		case 10:
			cnt_ture_arm = 0;
			mainBoard->servoA.On();//Arm close
			mainBoard->servoB.Off();
			mainBoard->servoC.On();
			break;
		case 3:
		case 7:
		case 11:
			if(completed_arm == false || cnt_ture_arm < 5) break;
			cnt_ture_arm = 0;
			cnt_ture_plate = 0;
			mainBoard->servoA.On();
			mainBoard->servoB.On();//Release object
			mainBoard->servoC.On();
			break;
		case 12:
			mainBoard->servoA.Off();
			if(completed_plate == false || cnt_ture_plate < 5) break;
			cnt_ture_plate = 0;
			mainBoard->servoB.On();
			mainBoard->servoC.Off();//plate capital
			break;
		default:
			mode = -2;
			cnt_ture_arm = 0;
			mainBoard->servoA.On();
			mainBoard->servoB.On();
			mainBoard->servoC.On();
			break;
		}
	}

	int pwmArm(int potentiometer){
		int target_height = 0;

		switch(mode){
		case -2:
			target_height = HEIGHT_DEF;
			break;
		case -1:
			target_height = HEIGHT_DEF + 500;
			break;
		//Arm open
		case 0:
		case 4:
		case 8:
			target_height = HEIGHT_STAY;
			break;
		//Get object
		case 1:
		case 5:
		case 9:
			target_height = HEIGHT_GET;
			break;
		//Arm close
		case 2:
			target_height = HEIGHT_OB1 + 200;
			break;
		case 6:
			target_height = HEIGHT_OB2 + 200;
			break;
		case 10:
			target_height = HEIGHT_OB3 + 200;
			break;
		//Release object
		case 3:
			target_height = HEIGHT_OB1;
			break;
		case 7:
			target_height = HEIGHT_OB2;
			break;
		case 11:
			target_height = HEIGHT_OB3;
			break;
		case 12://plate capital
			target_height = HEIGHT_TOP;
			break;
		default:
			break;
		}

		if(abs(target_height - potentiometer) < 15){
			completed_arm = true;
			pwm_arm = 50;
		}
		else {
			completed_arm = false;
			pwm_arm += ((target_height - potentiometer) - pwm_arm) / 5;
		}

		if(abs(potentiometer_old - potentiometer) < 20 && completed_arm == false){
			if(target_height - potentiometer >= 0) pwm_arm += 10;
			else pwm_arm -= 10;
		}

		potentiometer_old = potentiometer;

		//Pass touch object
		if(pause_time < 10 && (mode % 4 == 0)){ //builder_mode:0,4,8,12
			pwm_arm = 250 - 5 * pause_time;
			pause_time++;
		}

		return pwm_arm * 2;
	}

	void resetPause(){
		pause_time = 0;
	}

	int pwmPlate(int potentiometer){
		int target = 0;

		if(mode == 12){
			target = PLATE_TOP;
			if(abs(PLATE_TOP - potentiometer) < 20) completed_plate = true;
		}else{
			target = PLATE_BOTTOM;
			completed_plate = false;
		}

		pwm_plate += ((target - potentiometer) - pwm_plate) / 5;

		if(pause_time_plate < 20) pwm_plate = 0;

		if(abs(target - potentiometer) < 400){
			setLimit(pwm_plate,abs(target - potentiometer) / 2 + 50);
			pause_time_plate = 0;
		}
		else if(pause_time_plate < 25){
			pause_time_plate++;
		}

		return pwm_plate;
	}

	int getGain(){
		int ps3_gain = PS3_A_GAIN;

		switch(mode){
		case -2:
		case -1:
			break;
		//Arm open
		case 0:
		case 4:
		case 8:
			ps3_gain = 1;
			break;
		//Get object
		case 1:
		case 5:
		case 9:
			ps3_gain = 0;
			break;
		//Arm close
		case 2:
			break;
		case 6:
			ps3_gain = 2;
			break;
		case 10:
			ps3_gain = 1;
			break;
		//Release object
		case 3:
			break;
		case 7:
			ps3_gain = 2;
			break;
		case 11:
			ps3_gain = 1;
			break;
		case 12://plate capital
			ps3_gain = 1;
			break;
		}

		return ps3_gain;
	}

	int getMode(){
		return mode;
	}

	bool getComp(int mode){
		if(mode == PLATE) return completed_plate;
		else return completed_arm;
	}

	void Reset(){
		mode = -2;
		changeMode(-2);
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

void setLimitFloat(float &value,float limit){
	value = min(max(value, - limit), limit);
}
