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


#define RAD_TO_DEG (180/M_PI)
#define R_LIMIT 200
#define R_Gyro_LIMIT 200
#define SPEED_Linearly 150


int auto_runXY[2]  = {0},
	anti_slip[2] = {0};


int motorDriver_Protecter(int IN_Mout);
int ps3Analog_ValueChanger(int IN_100);
int Anti_sliper(int XY_100, int i);
int Auto_Runner(int t_e, int x_or_y, int enc_old, int enc_now);


int main(void) {
	float encX      = 0,
		  encY      = 0,
		  encX_now  = 0,
		  encY_now  = 0,
		  encX_old  = 0,
		  encY_old  = 0,
		  yaw_now   = 0,
		  yaw_old   = 0,
		  yaw_value = 0,
		  yaw_reset = 0,
		  rotation  = 0,
		  targetX   = 0,
		  targetY   = 0;

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

	while (1) {
		int rotation_gyro = 0,
			A_v           = 0,
			B_v           = 0,
			C_v           = 0,
			A_out         = 0,
			B_out         = 0,
			C_out         = 0,
			X_100         = 0,
			Y_100         = 0;

		mainBoard.can.Update();

		//Safety start
		if (Start == false || ps3con->getButtonPress(CONNECTED) == 0) {
			if (ps3con->getButtonPress(START)) Start = true;

			if (Start == true) mainBoard.buzzer.set(-1, 6);
			else               mainBoard.buzzer.stop();

			mainBoard.led.Flash();
			char str[128];
			sprintf(str, "Please push START button. \r\n");
			debug << str;
			MillisecondTimer::delay(50);
			continue;
		} else
			mainBoard.buzzer.stop();

		mainBoard.led.On();

		encX_now = mainBoard.encoders.getCounter1();
		encY_now = mainBoard.encoders.getCounter2();

		if (encX_now - encX_old < 10000 && encX_now - encX_old > -10000) {
			encX += encX_now - encX_old;
		}
		if (encY_now - encY_old < 10000 && encY_now - encY_old > -10000) {
			encY += encY_now - encY_old;
		}

		//Set Target of AutoRUN
		if (ps3con->getButtonPress(TRIANGLE)) {
			targetX = encX;
			targetY = encY;
		}

		sensor_timer.ahrs.getYaw(yaw_now);
		yaw_now *= RAD_TO_DEG;

		//Gyro Filter
		if (yaw_now - yaw_old > 0.05 || yaw_now - yaw_old < -0.05) yaw_value += yaw_now - yaw_old;

		rotation_gyro = (yaw_value - yaw_reset) * 17;

		if (ps3con->getButtonPress(CIRCLE)) {
			yaw_reset = yaw_value;
			rotation_gyro = 0;
		}

		if(encX_now - encX_old == 0){
			if      (rotation_gyro > 0) rotation_gyro += 5;
			else if (rotation_gyro < 0) rotation_gyro -= 5;
		}

		if (rotation_gyro >  R_Gyro_LIMIT) rotation_gyro =  R_Gyro_LIMIT;
		if (rotation_gyro < -R_Gyro_LIMIT) rotation_gyro = -R_Gyro_LIMIT;

		//Main rotation
		rotation = ps3con->getAnalog(ANALOG_R2) - ps3con->getAnalog(ANALOG_L2);

		if (rotation >  R_LIMIT) rotation =  R_LIMIT;
		if (rotation < -R_LIMIT) rotation = -R_LIMIT;

		X_100 = ps3Analog_ValueChanger(ps3con->getAnalog(ANALOG_L_X));
		Y_100 = ps3Analog_ValueChanger(ps3con->getAnalog(ANALOG_L_Y));

		if (ps3con->getButtonPress(RIGHT)) X_100 =  SPEED_Linearly;
		if (ps3con->getButtonPress(LEFT))  X_100 = -SPEED_Linearly;
		if (ps3con->getButtonPress(UP))    Y_100 = -SPEED_Linearly;
		if (ps3con->getButtonPress(DOWN))  Y_100 =  SPEED_Linearly;

		//Automatically adjust the distance between the material
		if (ps3con->getButtonPress(SQUARE)) {
			int target_Obj = 1500;

			if (mainBoard.ad[0]->get() > target_Obj + 25 || mainBoard.ad[0]->get() < target_Obj - 25) {
				Y_100 = (mainBoard.ad[0]->get() - target_Obj) / 3;

				if(encY_now - encY_old == 0){
					if(Y_100 > 0)      Y_100 += 2;
					else if(Y_100 < 0) Y_100 -= 2;
				}
			}

			if (Y_100 > 90)  Y_100 = 90;
			if (Y_100 < -90) Y_100 = -90;
		}

		//auto RUN
		if (ps3con->getButtonPress(CROSS)) {
			X_100 = Auto_Runner(encX - targetX, 0, encX_old, encX_now);
			Y_100 = Auto_Runner(targetY - encY, 1, encY_old, encY_now);
		}

		X_100 = Anti_sliper(X_100, 0);
		Y_100 = Anti_sliper(Y_100, 1);

		//motor
		A_v = Y_100 * -1 + X_100 / 2 + rotation + rotation_gyro;
		B_v = Y_100 + X_100 / 2 + rotation + rotation_gyro;
		C_v = X_100 * -1 + rotation + rotation_gyro;

		A_out = motorDriver_Protecter(A_v);
		B_out = motorDriver_Protecter(B_v);
		C_out = motorDriver_Protecter(C_v);

		//Motor output
		mainBoard.motorA.setOutput((float) A_out / 500.0);
		mainBoard.motorB.setOutput((float) B_out / 500.0);
		mainBoard.motorC.setOutput((float) C_out / 500.0);

		yaw_old = yaw_now;
		encX_old = encX_now;
		encY_old = encY_now;

		//debug
		char str[128];
		sprintf(str, "%.5f\r\n", (float) mainBoard.ad[0]->get());
		//sprintf(str, "%.5f %.5f %.5f %.5f %.5f\r\n", (float) A_out, (float) B_out,
		//		(float)C_out, (float)X_100,(float)Y_100);
		debug << str;

		MillisecondTimer::delay(50);
	}

}

int ps3Analog_ValueChanger(int IN_100) {
	int ret;
	ret = (IN_100 - 127.5) * 0.7843;
	ret *= 4;

	if (ret < 14 && ret > -14) ret = 0;

	return ret;
}

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

			if      (autoXY[x_or_y] < 0) autoXY[x_or_y] -= 5;
			else if (autoXY[x_or_y] > 0) autoXY[x_or_y] += 5;

		} else {
			autoXY[x_or_y] += ((t_e / 15) - autoXY[x_or_y]) / 3;
		}
	} else {
		autoXY[x_or_y] = 0;
	}

	if      (autoXY[x_or_y] >=  250) autoXY[x_or_y] =  250;
	else if (autoXY[x_or_y] <= -250) autoXY[x_or_y] = -250;

	return autoXY[x_or_y];
}
/* End Of File ---------------------------------------------------------------*/
