/*
 ============================================================================
 Name        : main.cpp
 Author      : Kentaro Doi
 Version     : 1.0
 Copyright   : kentan.jp
 Description : SRC2015 Bteam Manual
 ============================================================================
 */

#include "hw_config.h"
#include "config/stm32plus.h"
#include "config/gpio.h"
#include "config/timing.h"
#include "utils/debug.h"
#include "board/main_v3.h"
#include "devices/can/ps3con.h"
#include "machine/machine.h"
#include "devices/buzzer.h"

using namespace stm32plus;


#define RAD_TO_DEG      (180/M_PI)
#define R_LIMIT         200
#define R_Gyro_LIMIT    200
#define SPEED_Linearly  150
#define PWM_LIMIT       500
#define WALL_DISTANCE   850
#define OBJECT_DISTANCE 900


int main(void) {
	enum{
		X,
		Y
	};
	int enc_now[2]  = {},
		enc_old[2]  = {};

	float yaw_now   = 0,
		  yaw_old   = 0;

	bool Start = false;

	//Initialise Systick
	MillisecondTimer::initialise();

	Nvic::initialise();

	MainV3 mainBoard;

	MillisecondTimer::delay(100);

	PS3Con *ps3con = new PS3Con();

	mainBoard.can.AddListenerNode(ps3con);

	mainBoard.mpu6050.setTimeout(20);

	while (!mainBoard.mpu6050.test())
		;
	mainBoard.mpu6050.setup();
	mainBoard.mpu6050.setGyrRange(mainBoard.mpu6050.GYR_RANGE_500DPS);

	SensorTimer sensor_timer(&mainBoard);

	Machine machine;
	Gyro gyro;

	while (1) {
		int rotation_gyro = 0,
			rotation      = 0,
			rotation_90   = 0,
			A_out         = 0,
			B_out         = 0,
			C_out         = 0,
			X_100         = 0,
			Y_100         = 0;

		mainBoard.can.Update();
		mainBoard.buzzer.stop();

		//Safety start
		if (Start == false || ps3con->getButtonPress(CONNECTED) == 0) {
			if (ps3con->getButtonPress(START)) Start = true;

			if (Start == true) mainBoard.buzzer.set(-1, 6);

			mainBoard.led.Flash();
			debug << "Please push START button. \r\n";
			MillisecondTimer::delay(50);
			continue;
		}

		mainBoard.led.On();

		enc_now[X] = mainBoard.encoders.getCounter1();
		enc_now[Y] = mainBoard.encoders.getCounter2();

		machine.extendEncValue(enc_now[X],enc_old[X],MODE_X);
		machine.extendEncValue(enc_now[Y],enc_old[Y],MODE_Y);

		//Set TargetLocation
		if (ps3con->getButtonPress(TRIANGLE)) machine.setTargetLocation();

		sensor_timer.ahrs.getYaw(yaw_now);
		yaw_now *= RAD_TO_DEG;
		gyro.set(yaw_now,yaw_old);

		rotation_gyro = gyro.correct();

		if (ps3con->getButtonPress(CIRCLE)) gyro.reset(rotation_gyro);

		//GYRO Add output when "cannot move"
		if (machine.checkMove(MODE_X) == false){
			if      (rotation_gyro > 0) rotation_gyro += 0;
			else if (rotation_gyro < 0) rotation_gyro -= 0;
		}

		setLimit(rotation_gyro,R_Gyro_LIMIT);

		//Rotate (angle of 90)
		if (ps3con->getButtonPress(R1))      rotation_90 = gyro.angle90(TURN_RIGHT);
		else if (ps3con->getButtonPress(L1)) rotation_90 = gyro.angle90(TURN_LEFT);
		else gyro.resetAngle90();

		//Rotate (Normal)
		rotation = ps3con->getAnalog(ANALOG_R2) - ps3con->getAnalog(ANALOG_L2);
		setLimit(rotation,R_LIMIT);

		X_100 = ps3con->convertValue(ps3con->getAnalog(ANALOG_L_X));
		Y_100 = ps3con->convertValue(ps3con->getAnalog(ANALOG_L_Y));

		if (ps3con->getButtonPress(RIGHT)) X_100 =  SPEED_Linearly;
		if (ps3con->getButtonPress(LEFT))  X_100 = -SPEED_Linearly;
		if (ps3con->getButtonPress(UP))    Y_100 = -SPEED_Linearly;
		if (ps3con->getButtonPress(DOWN))  Y_100 =  SPEED_Linearly;

		//Automatically adjust the distance between the object
		if (ps3con->getButtonPress(SQUARE) && ps3con->getButtonPress(UP)) {
			Y_100 =  machine.adjustDistance((int)mainBoard.ad[0]->get(),OBJECT_DISTANCE,MODE_Y);
			if(Y_100 == 0) mainBoard.buzzer.set(-1, 6);
		}

		//Automatically adjust the distance between the wall
		if (ps3con->getButtonPress(SQUARE) && ps3con->getButtonPress(LEFT)) {
			X_100 =  machine.adjustDistance((int)mainBoard.ad[1]->get(),WALL_DISTANCE,MODE_X);
			if(X_100 == 0) mainBoard.buzzer.set(-1, 6);
		}

		//Automatically run to target
		if (ps3con->getButtonPress(CROSS)) {
			X_100 = machine.runToTarget(MODE_X);
			Y_100 = machine.runToTarget(MODE_Y);

			if(X_100 == 0 && Y_100 == 0) mainBoard.buzzer.set(-1, 6);
		}

		X_100 = machine.antiSlip(X_100, MODE_X);
		Y_100 = machine.antiSlip(Y_100, MODE_Y);

		//motor
		A_out =  X_100 / 2 - Y_100 + rotation + rotation_gyro + rotation_90;
		B_out =  X_100 / 2 + Y_100 + rotation + rotation_gyro + rotation_90;
		C_out = -X_100             + rotation + rotation_gyro + rotation_90;

		setLimit(A_out,PWM_LIMIT);
		setLimit(B_out,PWM_LIMIT);
		setLimit(C_out,PWM_LIMIT);

		//PWM output
		mainBoard.motorA.setOutput((float) A_out / 500.0);
		mainBoard.motorB.setOutput((float) B_out / 500.0);
		mainBoard.motorC.setOutput((float) C_out / 500.0);

		yaw_old = yaw_now;
		enc_old[X] = enc_now[X];
		enc_old[Y] = enc_now[Y];

		//debug
		char str[128];
		sprintf(str, "%.5f\r\n",yaw_old);
		debug << str;

		MillisecondTimer::delay(50);
	}
}
/* End Of File ---------------------------------------------------------------*/
