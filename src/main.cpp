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

#include "machine/machine.h"

#include "devices/buzzer.h"
#include "devices/can/ps3con.h"
#include "devices/spimotor.h"

using namespace stm32plus;


#define RAD_TO_DEG      (180/M_PI)
#define R_LIMIT         200
#define R_Gyro_LIMIT    200
#define SPEED_Linearly  100
#define PWM_LIMIT       500

//#define debug cout


int main(void) {
	int enc_now[2]   = {},
		enc_old[2]   = {},
		push_time[2] = {};

	float yaw_now   = 0,
		  yaw_old   = 0;

	bool Start = false;

	//Initialise Systick
	MillisecondTimer::initialise();

	Nvic::initialise();

	MainV3 mainBoard;
	SpiMotor spimotor( *mainBoard.spi3v1, 1);

	MillisecondTimer::delay(100);

	PS3Con *ps3con = new PS3Con();

	mainBoard.can.AddListenerNode(ps3con);

	mainBoard.mpu6050.setTimeout(20);

	debug<<"[INFO]Testing MPU6050...\r\n";

	while(!mainBoard.mpu6050.test());
	debug<<"[INFO]MPU6050 test passed.\r\n";

	debug<<"[INFO]Setting up MPU6050...\r\n";
	mainBoard.mpu6050.setup();
	mainBoard.mpu6050.setGyrRange(mainBoard.mpu6050.GYR_RANGE_500DPS);
	debug<<"[INFO]Setup complete!\r\n";

	SensorTimer sensor_timer(&mainBoard);

	Machine machine;
	Gyro gyro;
	Builder build(&mainBoard,&machine);

	//Reset
	build.Reset();

	while (1) {
		int rotation_gyro = 0,
			rotation      = 0,
			D_out         = 0,
			SPI_out       = 0,
			X_100         = 0,
			Y_100         = 0;

		float A_out       = 0,
			  B_out       = 0,
			  C_out       = 0;

		mainBoard.can.Update();
		mainBoard.buzzer.stop();

		//Safety start
		if (Start == false || ps3con->getButtonPress(CONNECTED) == 0) {
			if (ps3con->getButtonPress(START)){
				Start = true;
				debug << "[INFO]Safety launch success!\r\n";
			}else{
				debug << "[WARN]Please push START button.\r\n";
			}

			if (Start == true) mainBoard.buzzer.set(-1, 6);
			else               mainBoard.buzzer.flash();

			mainBoard.led.flash();

			build.changeMode(NO_CAHNGE);
			D_out   = build.pwmArm((int)mainBoard.ad[2]->get());
			SPI_out = build.pwmPlate((int)mainBoard.ad[3]->get());
			setLimit(D_out,300);
			setLimit(SPI_out,200);
			mainBoard.motorD.setOutput((float) D_out / 500.0);
			spimotor.setOutput((float) SPI_out / 500.0);

			MillisecondTimer::delay(50);
			continue;
		}else if(Start == true && ps3con->getButtonPress(START)){//Emergency stop
			mainBoard.buzzer.set(-1, 4);
			debug << "[WARN]Emergency stop!!\r\n";
			Start = false;
			build.Reset();

			//PWM output
			mainBoard.motorA.setOutput(0 / 500.0);
			mainBoard.motorB.setOutput(0 / 500.0);
			mainBoard.motorC.setOutput(0 / 500.0);
			mainBoard.motorD.setOutput(0 / 500.0);
			spimotor.setOutput(0/500.0);

			MillisecondTimer::delay(200);
			continue;
		}

		mainBoard.led.On();

		enc_now[X] = mainBoard.encoders.getCounter1();
		enc_now[Y] = mainBoard.encoders.getCounter2();

		machine.extendEncValue(enc_now[X],enc_old[X],X);
		machine.extendEncValue(enc_now[Y],enc_old[Y],Y);

		sensor_timer.ahrs.getYaw(yaw_now);
		yaw_now *= RAD_TO_DEG;
		gyro.set(yaw_now,yaw_old);

		rotation_gyro = gyro.correct();

		setLimit(rotation_gyro,R_Gyro_LIMIT);

		//Rotate (angle of 90)
		if (ps3con->getButtonPress(R1) && push_time[0] == 0){
			mainBoard.buzzer.set(-1, 5);
			gyro.setAngle90(90);
			push_time[0] = 1;
		}
		else if (ps3con->getButtonPress(L1) && push_time[0] == 0){
			mainBoard.buzzer.set(-1, 5);
			gyro.setAngle90(-90);
			push_time[0] = 1;
		}

		//Rotate (Normal)
		if(ps3con->getButtonPress(L2)||ps3con->getButtonPress(R2)){
			gyro.reset(rotation_gyro);
			rotation = (ps3con->getAnalog(ANALOG_R2) - ps3con->getAnalog(ANALOG_L2)) / 2;
		}
		machine.antiSlip(rotation,ROTATE,NO_CAHNGE);
		setLimit(rotation,R_LIMIT);

		//Build capital (push_time)
		if(ps3con->getButtonPress(CIRCLE) && push_time[1] == 0){
			mainBoard.buzzer.set(-1, 5);
			push_time[1] = 1;
			if(build.getMode() == -2 && build.getStatusBIG() == 0) build.releaseBIG();
			else build.changeMode();
			build.resetPause();
		}else if(ps3con->getButtonPress(CROSS) && push_time[1] == 0){
			mainBoard.buzzer.set(-1, 4);
			push_time[1] = 1;
			build.changeMode(build.getMode() - 1);
			build.resetPause();
		}
		build.changeMode(NO_CAHNGE);

		for(int i = 0;i < 2;i++){
			if(push_time[i] != 0){
				push_time[i]++;
				if(push_time[i] > 10) push_time[i] = 0;
			}
		}

		X_100 = ps3con->convertValue(ps3con->getAnalog(ANALOG_L_X),build.getGain());
		Y_100 = ps3con->convertValue(ps3con->getAnalog(ANALOG_L_Y),build.getGain());

		//Adjust bit
		if(ps3con->getButtonPress(TRIANGLE)){
			if (ps3con->getButtonPress(RIGHT)){      if(X_100 <  SPEED_Linearly) X_100 += 20;}
			else if (ps3con->getButtonPress(LEFT)){  if(X_100 > -SPEED_Linearly) X_100 -= 20;}
			else if (ps3con->getButtonPress(UP)){    if(Y_100 > -SPEED_Linearly) Y_100 -= 20;}
			else if (ps3con->getButtonPress(DOWN)){  if(Y_100 <  SPEED_Linearly) Y_100 += 20;}
			else{ X_100 = 0; Y_100 = 0;                                                      }
		}

		//Automatically adjust the distance between the object
		if (ps3con->getButtonPress(SQUARE) && ps3con->getButtonPress(UP)) {
			Y_100 =  machine.adjustDistance((int)mainBoard.ad[0]->get(),build.getMode(),Y);
			if(Y_100 == 0) mainBoard.buzzer.set(-1, 6);
		}

		//Automatically adjust the distance between the wall
		if (ps3con->getButtonPress(SQUARE) && ps3con->getButtonPress(LEFT)) {
			X_100 =  machine.adjustDistance((int)mainBoard.ad[1]->get(),build.getMode(),X);
			if(X_100 == 0) mainBoard.buzzer.set(-1, 6);
		}

		//Automatically run to target
//		if (ps3con->getButtonPress(CROSS)) {
//			X_100 = machine.runToTarget(X);
//			Y_100 = machine.runToTarget(Y);
//
//			if(X_100 == 0 && Y_100 == 0) mainBoard.buzzer.set(-1, 6);
//		}

		X_100 = machine.antiSlip(X_100, X,build.getMode());
		Y_100 = machine.antiSlip(Y_100, Y,build.getMode());

		//motor
		A_out =  X_100 / 2 - Y_100 * sqrt(3) / 2 + rotation + rotation_gyro;
		B_out =  X_100 / 2 + Y_100 * sqrt(3) / 2 + rotation + rotation_gyro;
		C_out = -X_100                           + rotation + rotation_gyro;

		//int test = 0;
		//SPI_out = -ps3con->convertValue(ps3con->getAnalog(ANALOG_R_Y),5);
		D_out = build.pwmArm((int)mainBoard.ad[2]->get());
		SPI_out = build.pwmPlate((int)mainBoard.ad[3]->get());

		setLimitFloat(A_out,PWM_LIMIT);
		setLimitFloat(B_out,PWM_LIMIT);
		setLimitFloat(C_out,PWM_LIMIT);
		setLimit(D_out,PWM_LIMIT);
		setLimit(SPI_out,120);

		//PWM output
		mainBoard.motorA.setOutput(A_out / 500.0);
		mainBoard.motorB.setOutput(B_out / 500.0);
		mainBoard.motorC.setOutput(C_out / 500.0);
		mainBoard.motorD.setOutput((float)D_out / 500.0);
		spimotor.setOutput((float)SPI_out/500.0);

		yaw_old    = yaw_now;
		enc_old[X] = enc_now[X];
		enc_old[Y] = enc_now[Y];

		//debug
		char str[128];
//			sprintf(str, "[DEBUG]Mode:%d Arm,Plate:%s,%s PS3con:%d,%d omniPWM:%.0f,%.0f,%.0f Gyro:%d Rotate:%d [TEST]%d %d\r\n"
//				,build.getMode(),build.getComp(ARM)?"O":"X",build.getComp(PLATE)?"O":"X"
//						,X_100,Y_100,A_out,B_out,C_out,rotation_gyro,rotation,(int)mainBoard.ad[3]->get(),SPI_out);
		sprintf(str,"[TEST]%d %d\r\n",(int)mainBoard.ad[3]->get(),SPI_out);
		//sprintf(str,"[TEST]Front:%d Left:%d\r\n",(int)mainBoard.ad[0]->get(),(int)mainBoard.ad[1]->get());
		debug << str;

		MillisecondTimer::delay(50);
	}
}
/* End Of File ---------------------------------------------------------------*/
