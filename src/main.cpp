/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"

#include "config/stm32plus.h"
#include "config/gpio.h"
#include "config/timing.h"
#include "utils/debug.h"
#include "board/main_v3.h"
#include "devices/can/ps3con.h"
#include "machine/machine.h"

#include "utils/MadgwickAHRS.h"

using namespace stm32plus;

/* Defines -------------------------------------------------------------------*/

#define RAD_TO_DEG (180/M_PI)

/* Variables -----------------------------------------------------------------*/
float yaw_now = 0, yaw_old = 0, yaw_value = 0, yaw_reset = 0, rotation = 0;

float targetX = 0, targetY = 0;

int A_out = 0, B_out = 0, C_out = 0, autoX = 0, autoY = 0;

bool Start = false;

/* Constants -----------------------------------------------------------------*/

//pin29
/* Function prototypes -------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/**************************************************************************/
/*!
 @brief  Main Program.
 @param  None.
 @retval None.
 */
/**************************************************************************/

int motorDriver_Protecter(int IN_Mout);
int ps3Analog_ValueChanger(int IN_100);

int main(void) {
	float encoderX = 0, encoderY = 0;
	float encoderX_now = 0, encoderX_old = 0, encoderY_now = 0,
			encoderY_old = 0;

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
		int X_raw = 0, Y_raw = 0, rotation_sub = 0, A_v = 0, B_v = 0, C_v = 0,
				X_100 = 0, Y_100 = 0;

		mainBoard.can.Update();

		//Safety start
		if (Start == false) {
			if (ps3con->getButtonPress(START))
				Start = true;

			mainBoard.led.Flash();
			char str[128];
			sprintf(str, "Please push START button. \r\n");
			debug << str;
			MillisecondTimer::delay(50);
			continue;
		}


		mainBoard.led.On();

		encoderX_now = mainBoard.encoders.getCounter1();
		encoderY_now = mainBoard.encoders.getCounter2();

		if (encoderX_now - encoderX_old < 500
				&& encoderX_now - encoderX_old > -500) {
			encoderX += encoderX_now - encoderX_old;

		}
		if (encoderY_now - encoderY_old < 500
				&& encoderY_now - encoderY_old > -500) {
			encoderY += encoderY_now - encoderY_old;

		}

		if (ps3con->getButtonPress(TRIANGLE)) {
			targetX = encoderX;
			targetY = encoderY;
		}

		yaw_now = sensor_timer.yaw;
		//sensor_timer.ahrs.getYawPitchRoll(yaw_now);

		if (yaw_now - yaw_old > 0.03 || yaw_now - yaw_old < -0.03) {
			yaw_value += yaw_now - yaw_old;
		}
		rotation_sub = (yaw_value - yaw_reset) * 15;
		if (ps3con->getButtonPress(CIRCLE)) {
			mainBoard.buzzer.setOutput((float) -500 / 500.0);
			yaw_reset = yaw_value;
			rotation_sub = 0;
		}else{
			mainBoard.buzzer.setOutput((float) 0 / 500.0);
		}

		if (rotation_sub > 200)
			rotation_sub = 200;
		if (rotation_sub < -200)
			rotation_sub = -200;

		if (ps3con->getButtonPress(R1)) {
			if (rotation < 200)
				rotation += 50;
			else
				rotation = 200;
		}

		if (ps3con->getButtonPress(L1)) {
			if (rotation > -200)
				rotation -= 50;
			else
				rotation = -200;
		}
		if (ps3con->getButtonPress(L1) == 0
				&& ps3con->getButtonPress(R1) == 0) {
			if (rotation <= 1 && rotation >= -1)
				rotation = 0;
			else if (rotation > 1)
				rotation -= 50;
			else if (rotation < -1)
				rotation += 50;
		}

		X_raw = ps3con->getAnalog(ANALOG_L_X);
		Y_raw = ps3con->getAnalog(ANALOG_L_Y);
		X_100 = ps3Analog_ValueChanger(X_raw);
		Y_100 = ps3Analog_ValueChanger(Y_raw);

		if (ps3con->getButtonPress(RIGHT)) {
			X_100 = 100;
		}
		if (ps3con->getButtonPress(LEFT)) {
			X_100 = -100;
		}
		if (ps3con->getButtonPress(UP)) {
			Y_100 = -100;
		}
		if (ps3con->getButtonPress(DOWN)) {
			Y_100 = 100;
		}

		//auto RUN
		if (ps3con->getButtonPress(CROSS)) {
			int t_eX = 0, t_eY = 0;
			t_eX = targetX - encoderX;
			t_eY = targetY - encoderY;

			if (t_eX >= 10 || t_eX <= -10) {
//				autoX += ((t_eX / 4) - autoX) / 5;
				if (t_eX / 3 >= 290) {
					autoX += 10;
				} else if (t_eX / 3 <= -290) {
					autoX -= 10;
				} else {
					autoX = t_eX / 3;
				}
			}
			if (t_eY >= 10 || t_eY <= -10) {
//				autoY += ((t_eY / 4) - autoY) / 5;
				if (t_eY / 3 >= 290) {
					autoY += 10;
				} else if (t_eY / 3 <= -290) {
					autoY -= 10;
				} else {
					autoY = t_eY / 3;
				}
			}
			X_100 = autoX;
			Y_100 = autoY;
		}

		A_v = B_v = C_v = 0;

		//moto
		A_v = Y_100 * -1 + X_100 / 2 + rotation + rotation_sub;
		B_v = Y_100 + X_100 / 2 + rotation + rotation_sub;
		C_v = X_100 * -1 + rotation + rotation_sub;

		//smooth
		/*
		 A_out += (A_v - A_out) / 5;
		 B_out += (B_v - B_out) / 5;
		 C_out += (C_v - C_out) / 5;
		 */
		A_out = motorDriver_Protecter(A_v);
		B_out = motorDriver_Protecter(B_v);
		C_out = motorDriver_Protecter(C_v);

		mainBoard.motorA.setOutput((float) A_out / 500.0);
		mainBoard.motorB.setOutput((float) B_out / 500.0);
		mainBoard.motorC.setOutput((float) C_out / 500.0);

		yaw_old = yaw_now;

		encoderX_old = encoderX_now;
		encoderY_old = encoderY_now;

		char str[128];

		//sprintf(str, "AD: %5d %5d %5d %5d %5d %5d %5d %5d \r\n", mainBoard.ad[0]->get(), mainBoard.ad[1]->get(), mainBoard.ad[2]->get(), mainBoard.ad[3]->get(), mainBoard.ad[4]->get(), mainBoard.ad[5]->get(), mainBoard.ad[6]->get(), mainBoard.ad[7]->get());
		//sprintf(str, "%.5f %d\r\n", yaw_value,rotation_sub);
		sprintf(str, "YPR: %.5f\r\n", (float) A_out / 500.0);
		debug << str;

		MillisecondTimer::delay(50);
	}

}

int ps3Analog_ValueChanger(int IN_100) {
	IN_100 = (IN_100 - 127.5) * 0.7843;
	IN_100 *= 3;

	if (IN_100 < 10 && IN_100 > -10)
		IN_100 = 0;

	return IN_100;
}

int motorDriver_Protecter(int IN_Mout) {
	int ret;
	//if (IN_out < 3 && IN_out > -3)
	//	ret = 0;
	if (IN_Mout > 499)
		ret = 499;
	else if (IN_Mout < -499)
		ret = -499;
	else
		ret = IN_Mout;
	return ret;
}

int smooth_Controller(int IN_Sout) {

	return IN_Sout;
}

/* End Of File ---------------------------------------------------------------*/
