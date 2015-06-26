/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"

#include "config/stm32plus.h"
#include "config/gpio.h"
#include "config/timing.h"
#include "utils/debug.h"

#include "board/main_v3.h"

#include "devices/spimotor.h"

#include "devices/can/ps3con.h"

#include "utils/MadgwickAHRS.h"

#include "machine/machine.h"

using namespace stm32plus;

/* Defines -------------------------------------------------------------------*/

#define RAD_TO_DEG (180/M_PI)

/* Variables -----------------------------------------------------------------*/
float yaw_now, yaw_old, yaw_value;
float yaw, pitch, roll;

int X_raw, Y_raw, X_v, Y_v, rotation, rotation_sub,A_out = 0, B_out, C_out, A_v, B_v, C_v,X_100, Y_100;

int16_t acc[3] = { 0 };
int16_t gyr[3] = { 0 };

/* Constants -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

class SensorTimer {
public:
	MainV3 *mainBoard;
	MadgwickAHRS ahrs;

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

/**************************************************************************/
/*!
 @brief  Main Program.
 @param  None.
 @retval None.
 */
/**************************************************************************/

int motorDriver_Protecter(int IN_out);
int ps3Analog_ValueChanger(float IN_100);

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
		mainBoard.can.Update();
		mainBoard.led.On();

		//float q[4];
		//sensor_timer.ahrs.getQuaternion(q);
		if (ps3con->getButtonPress(CIRCLE)) {
			for (int i; i <= 2; i++) {
				acc[i] = 0;
				gyr[i] = 0;
			}
		}

		sensor_timer.ahrs.getYawPitchRoll(yaw, pitch, roll);

		yaw_now = yaw * RAD_TO_DEG;

		if (yaw_now - yaw_old > 0.005 || yaw_now - yaw_old < -0.005)
			yaw_value += yaw_now - yaw_old;

		rotation_sub = yaw_value * 20;
		if (rotation_sub > 100)
			rotation_sub = 100;
		if (rotation_sub < -100)
			rotation_sub = -100;

		if (ps3con->getButtonPress(R1)) {
			if (rotation < 200)
				rotation += 1;
			else
				rotation = 200;
		}

		if (ps3con->getButtonPress(L1)) {
			if (rotation > -200)
				rotation -= 1;
			else
				rotation = -200;
		}
		if (ps3con->getButtonPress(L1) == 0
				&& ps3con->getButtonPress(R1) == 0) {
			if (rotation <= 1 && rotation >= -1)
				rotation = 0;
			else if (rotation > 1)
				rotation -= 1;
			else if (rotation < -1)
				rotation += 1;
		}

		X_raw = ps3con->getAnalog(ANALOG_L_X);
		Y_raw = ps3con->getAnalog(ANALOG_L_Y);
		X_100 = ps3Analog_ValueChanger(X_raw);
		Y_100 = ps3Analog_ValueChanger(Y_raw);

		A_v = B_v = C_v = 0;

		//motor
		A_v = (int) Y_100 * -1 + (int) X_100 / 2 + rotation + rotation_sub;
		B_v = (int) Y_100 + (int) X_100 / 2 + rotation + rotation_sub;
		C_v = (int) X_100 * -1 + rotation + rotation_sub;

		if (A_out < A_v) {
			A_out++;
		} else
			A_out--;

		if (B_out < B_v)
			B_out++;
		else
			B_out--;

		if (C_out < C_v)
			C_out++;
		else
			C_out--;

		A_out = motorDriver_Protecter(A_out);
		B_out = motorDriver_Protecter(B_out);
		C_out = motorDriver_Protecter(C_out);

		//mainBoard.motorA.setOutput((float)A_out / 500.0);
		//mainBoard.motorB.setOutput((float)B_out / 500.0);
		//mainBoard.motorC.setOutput((float)C_out / 500.0);

		yaw_old = yaw_now;

		char str[128];
		//sprintf(str, "YPR: %d %.5f %.5f\r\n", (int) yaw_value,
		//pitch * RAD_TO_DEG, roll * RAD_TO_DEG
		//);
		sprintf(str, "Motor out: %.5f %.5f %.5f %.5f\r\n", (float)A_v, (float)A_out,(float)motorDriver_Protecter(A_out), (float)X_100);

		debug << str;

		MillisecondTimer::delay(50);
	}

}

int ps3Analog_ValueChanger(float IN_100) {

	IN_100 = (IN_100 - 127.5) * 0.7843;
	IN_100 *= 2;

	if (IN_100 < 10 && IN_100 > -10)
		IN_100 = 0;

	return (int)IN_100;
}

int motorDriver_Protecter(int IN_out) {

	if (IN_out < 3 && IN_out > -3)
		IN_out = 0;
	if (IN_out > 500)
		IN_out = 500;
	if (IN_out < -500)
		IN_out = -500;

	return IN_out;
}

/* End Of File ---------------------------------------------------------------*/
