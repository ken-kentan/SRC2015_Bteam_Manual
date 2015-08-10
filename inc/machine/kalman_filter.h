#pragma once

#include "config/stm32plus.h"
#include "board/main_v3.h"
#include "control/controlTimer.h"
#include "kalman.h"

/* -- Defines -- */
#ifndef M_PI
#define M_PI 3.1415926535
#endif

Kalman kalman;

using namespace stm32plus;

namespace stm32plus{

class KalmanFilter : public MainV3{
private:
	ControlTimer<
	Timer6<Timer6InternalClockFeature,Timer6InterruptFeature>,
	KalmanFilter
	> mainCtrlTimer;

	uint32_t enc_p[3] = {0};
	uint32_t enc_n[3] = {0};
	int32_t speed[3] = {0};
	int32_t speed_p[3] = {0};
	float GyrZ = 0.0f;
	float rate = 0.0f;
	float yaw=0.0f;
	float kalman_yaw = 0.0f;
	int16_t offset_GyrZ = 0;

public:
	KalmanFilter():MainV3(),
	mainCtrlTimer(this,100,2)
{
		mainCtrlTimer.bind(&KalmanFilter::ctrlHandler);
}
	//割り込みハンドラ
	void ctrlHandler(TimerEventType te,uint8_t timerNumber){

		//encoder
		enc_n[0] = encoders.getCounter1();
		enc_n[1] = encoders.getCounter2();
		enc_n[2] = encoders.getCounter3();

		speed[0] = (int32_t)(enc_n[0] - enc_p[0]);
		speed[1] = (int32_t)(enc_n[1] - enc_p[1]);
		speed[2] = (int32_t)(enc_n[2] - enc_p[2]);
		for(int i=0;i<3;i++){
			if(speed[i] > 30000 || speed[i] < -30000)
				speed[i] = speed_p[i];
			enc_p[i] = enc_n[i];
			speed_p[i] = speed[i];
		}

		//mpu
		/*
		GyrZ = mpu6050.readGyrZ() - offset_GyrZ;
		if( ( GyrZ > -5 ) && ( GyrZ < 5 ) )
			GyrZ = 0;
		yaw += (float)GyrZ/32768.0f * 500.0f*(M_PI/180.0f) * 4.0;
		*/
		GyrZ = (float)(mpu6050.readGyrZ() - offset_GyrZ);
		if( ( GyrZ > -5 ) && ( GyrZ < 5 ) )
			GyrZ = 0;
		rate = GyrZ/131.0;
		yaw = GyrZ/32768.0f * 500.0f*(M_PI/180.0f);
		kalman_yaw += kalman.getAngle(yaw,rate,1);

	}
	//func
	int32_t GetSpeed(int i){
		return speed[i];
	}
	float getState_yaw(){
		return yaw;
	}
	float getKalmanYaw(){
		return kalman_yaw;
	}
	int16_t getState_gyrZ(){
		return GyrZ;
	}
	int16_t getOffset_GyrZ(){
		return offset_GyrZ;
	}
	void setOffset(int16_t offset){
		offset_GyrZ = offset;
	}
	void ResetState(){
		kalman_yaw=0;
	}
};

}
