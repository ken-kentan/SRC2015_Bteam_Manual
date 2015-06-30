/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"

#include "config/stm32plus.h"
#include "config/gpio.h"
#include "config/timing.h"
#include "utils/debug.h"
#include "board/main_v3.h"
#include "devices/spimotor.h"
#include "devices/can/ps3con.h"
#include "machine/machine.h"

using namespace stm32plus;


/* Defines -------------------------------------------------------------------*/

#define RAD_TO_DEG (180/M_PI)

/* Variables -----------------------------------------------------------------*/
float yaw_now = 0, yaw_old = 0, yaw_value = 0, yaw_reset = 0, rotation = 0;

int A_out = 0, B_out = 0, C_out = 0;

/* Constants -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/**************************************************************************/
/*!
 @brief  Main Program.
 @param  None.
 @retval None.
 */
/**************************************************************************/

int motorDriver_Protecter(int IN_out);
int ps3Analog_ValueChanger(int IN_100);

int main(void) {

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
		mainBoard.led.On();

		yaw_now = sensor_timer.yaw;
		//sensor_timer.ahrs.getYawPitchRoll(yaw_now);

		if (yaw_now - yaw_old > 0.05 || yaw_now - yaw_old < -0.05) {
			yaw_value += yaw_now - yaw_old;
		}
		rotation_sub = (yaw_value - yaw_reset) * 10;
		if (ps3con->getButtonPress(CIRCLE)) {
			yaw_reset = yaw_value;
			rotation_sub = 0;
		}


		if (rotation_sub > 100)
			rotation_sub = 100;
		if (rotation_sub < -100)
			rotation_sub = -100;

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

		A_v = B_v = C_v = 0;

		//moto
		A_v = Y_100 * -1 + X_100 / 2 + rotation + rotation_sub;
		B_v = Y_100 + X_100 / 2 + rotation + rotation_sub;
		C_v = X_100 * -1 + rotation + rotation_sub;

		A_out = A_v;
		B_out = B_v;
		C_out = C_v;

		A_out = motorDriver_Protecter(A_out);
		B_out = motorDriver_Protecter(B_out);
		C_out = motorDriver_Protecter(C_out);

		mainBoard.motorA.setOutput((float) A_out / 500.0);
		mainBoard.motorB.setOutput((float) B_out / 500.0);
		mainBoard.motorC.setOutput((float) C_out / 500.0);

		yaw_old = yaw_now;
/*
		 char str[128];

		 //sprintf(str, "YPR: %d %.5f %.5f\r\n", (int) yaw_value,
		 //pitch * RAD_TO_DEG, roll * RAD_TO_DEG
		 //);
		 sprintf(str, "Motor out: %.5f %.5f %.5f %.5f %.5f\r\n", (float) B_out,
		 (float) A_out, (float) C_out, (float) yaw_value,
		 (float) sensor_timer.yaw);

		 sprintf(str,"%.5f %.5f \r\n",yaw_now ,yaw_value);
		 debug << str;
*/
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

int motorDriver_Protecter(int IN_out) {
	int ret;
	//if (IN_out < 3 && IN_out > -3)
	//	ret = 0;
	if (IN_out > 450)
		ret = 450;
	else if (IN_out < -450)
		ret = -450;
	else
		ret = IN_out;
	return ret;
}

/* End Of File ---------------------------------------------------------------*/
