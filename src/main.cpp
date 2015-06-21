/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"

#include "config/stm32plus.h"
#include "config/gpio.h"
#include "config/timing.h"
#include "utils/debug.h"

#include "board/main_v3.h"

#include "devices/spimotor.h"

#include "devices/can/ps3con.h"
#include "devices/can/canair.h"

#include "utils/MadgwickAHRS.h"

using namespace stm32plus;

/* Defines -------------------------------------------------------------------*/
#ifndef M_PI
#define M_PI 3.1415926535
#endif

#define RAD_TO_DEG (180/M_PI)

/* Variables -----------------------------------------------------------------*/
float yaw_now, yaw_old, yaw_value;
float yaw, pitch, roll;

int X_raw = 0, Y_raw = 0, X_v = 0, Y_v = 0, rotation = 0,rotation_sub, A_out = 0, B_out = 0,
		C_out = 0, X_encoder, Y_encoder, X_encoder_now, Y_encoder_now,
		X_encoder_old, Y_encoder_old, X_encoder_BASE, Y_encoder_BASE;

float X_100, Y_100, A_v = 0, B_v = 0, C_v = 0;

/* Constants -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

class SensorTimer {
public:
	MainV3 *mainBoard;
	MadgwickAHRS ahrs;

	float yaw, pitch, roll;

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

/**************************************************************************/
/*!
 @brief  Main Program.
 @param  None.
 @retval None.
 */
/**************************************************************************/

int main(void) {

	//Initialise Systick
	MillisecondTimer::initialise();

	Nvic::initialise();

	MainV3 mainBoard;
	SpiMotor spimotor(*mainBoard.spi3v1, 0);

	MillisecondTimer::delay(100);

	/*
	CanAir canair(&mainBoard.can);

	canair.Set(0);
	canair.Update();
	MillisecondTimer::delay(1000);
	canair.Reset(0);
	canair.Update();
	*/

	PS3Con *ps3con = new PS3Con();

	mainBoard.can.AddListenerNode(ps3con);

	mainBoard.mpu6050.setTimeout(20);

	debug << "Testing MPU6050...\r\n";
	while (!mainBoard.mpu6050.test())
		;
	debug << "MPU6050 test passed.\r\n";

	debug << "Setting up MPU6050...\r\n";
	mainBoard.mpu6050.setup();
	mainBoard.mpu6050.setGyrRange(mainBoard.mpu6050.GYR_RANGE_500DPS);
	debug << "complete.\r\n";

	SensorTimer sensor_timer(&mainBoard);


	while (1) {
		mainBoard.can.Update();
		mainBoard.led.On();

		float q[4];
		sensor_timer.ahrs.getQuaternion(q);

		char str[128];

		sensor_timer.ahrs.getYawPitchRoll(yaw, pitch, roll);

		if (ps3con->getButtonPress(CIRCLE)) sensor_timer.ahrs.resetYaw();

		yaw_now = yaw * RAD_TO_DEG;

		if (yaw_now - yaw_old > 0.005 || yaw_now - yaw_old < -0.005)
			yaw_value += yaw_now - yaw_old;

		rotation_sub = yaw_value * 20;
		if(rotation_sub > 100)rotation_sub = 100;
		if(rotation_sub < -100)rotation_sub = -100;

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

		A_v = 0;
		B_v = 0;
		C_v = 0;
		X_raw = ps3con->getAnalog(ANALOG_L_X);
		Y_raw = ps3con->getAnalog(ANALOG_L_Y);

		X_100 = (X_raw - 127.5) * 0.7843;
		Y_100 = (Y_raw - 127.5) * 0.7843;

		X_100 *= 3;
		Y_100 *= 3;

		if (X_100 < 10 && X_100 > -10)
			X_100 = 0;
		if (Y_100 < 10 && Y_100 > -10)
			Y_100 = 0;

		//motor
		A_v = (int) Y_100 * -1 + (int) X_100 / 2 + rotation + rotation_sub;
		B_v = (int) Y_100 + (int) X_100 / 2 + rotation + rotation_sub;
		C_v = (int) X_100 * -1 + rotation + rotation_sub;

		if (A_out < A_v)
			A_out++;
		else
			A_out--;

		if (B_out < B_v)
			B_out++;
		else
			B_out--;

		if (C_out < C_v)
			C_out++;
		else
			C_out--;

		//sprintf(str, "YPR: %d %.5f %.5f\r\n", (int) yaw_value,
				//pitch * RAD_TO_DEG, roll * RAD_TO_DEG);
		//sprintf(str, "YPR: %.5f\r\n", yaw_value);

		//debug << str;

		mainBoard.motorA.setOutput(A_out / 500.0);
		mainBoard.motorB.setOutput(B_out / 500.0);
		mainBoard.motorC.setOutput(C_out / 500.0);

		yaw_old = yaw_now;

		MillisecondTimer::delay(2);
	}

}

/* End Of File ---------------------------------------------------------------*/
