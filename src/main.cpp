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

using namespace stm32plus;

/* Defines -------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

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
int X_raw = 0,Y_raw = 0,X_v = 0,Y_v = 0,rotation = 0,A_out=0,B_out=0,C_out=0,
		X_encoder,Y_encoder,X_encoder_now,Y_encoder_now,X_encoder_old,Y_encoder_old,X_encoder_BASE,Y_encoder_BASE;

float X_100,Y_100,A_v = 0 ,B_v = 0 ,C_v = 0;

int main(void)
{

  //Initialise Systick
  MillisecondTimer::initialise();

  Nvic::initialise();

  MainV3 mainBoard;
  SpiMotor spimotor( *mainBoard.spi3v1, 0);

  MillisecondTimer::delay(100);

  CanAir canair(&mainBoard.can);

   canair.Set(0);
   canair.Update();
   MillisecondTimer::delay(1000);
   canair.Reset(0);
   canair.Update();

   PS3Con *ps3con = new PS3Con();

   mainBoard.can.AddListenerNode(ps3con);

   X_encoder_BASE = mainBoard.encoders.getCounter1();
   Y_encoder_BASE = mainBoard.encoders.getCounter1();

   mainBoard.mpu6050.setTimeout(20);

     debug<<"Testing MPU6050...\r\n";
     while(!mainBoard.mpu6050.test());
     debug<<"MPU6050 test passed.\r\n";

     debug<<"Setting up MPU6050...\r\n";
     mainBoard.mpu6050.setup();
     debug<<"complete.\r\n";

  while(1){

	  mainBoard.can.Update();
      mainBoard.led.On();

      char str[128];

      if(ps3con->getButtonPress(R1)){
    	  //X_encoder_BASE = mainBoard.encoders.getCounter1();
    	  //Y_encoder_BASE = mainBoard.encoders.getCounter1();
    	  if(rotation < 200) rotation += 1;
    	  else rotation = 200;
      }

      if(ps3con->getButtonPress(L1)){
          	  //X_encoder_BASE = mainBoard.encoders.getCounter1();
          	  //Y_encoder_BASE = mainBoard.encoders.getCounter1();
          	  if(rotation > -200) rotation -= 1;
          	  else rotation = -200;
       }
      if(ps3con->getButtonPress(L1) == 0 && ps3con->getButtonPress(R1) == 0){
    	  if(rotation <= 1 && rotation >= -1) rotation = 0;
    	  else if(rotation > 1) rotation -= 1;
    	  else if(rotation < -1) rotation += 1;
      }

      A_v = 0 ;B_v = 0 ;C_v = 0;
      X_raw = ps3con->getAnalog(ANALOG_L_X);
      Y_raw = ps3con->getAnalog(ANALOG_L_Y);

      X_encoder_now = mainBoard.encoders.getCounter1() - X_encoder_BASE;
      Y_encoder_now = mainBoard.encoders.getCounter2() - Y_encoder_BASE;

      if(X_encoder_now >= X_encoder_old) X_encoder = (X_encoder_now - X_encoder_old);
      if(Y_encoder_now >= Y_encoder_old) Y_encoder = (Y_encoder_now - Y_encoder_old);
      if(X_encoder_now < X_encoder_old) X_encoder = (X_encoder_old - X_encoder_now);
      if(Y_encoder_now < Y_encoder_old) Y_encoder = (Y_encoder_old - Y_encoder_now);

      if(X_encoder > 500) X_encoder = 0;
      if(Y_encoder > 500) Y_encoder = 0;

      X_100 = (X_raw - 127.5) * 0.7843;
      Y_100 = (Y_raw - 127.5) * 0.7843;

      X_100 *= 3;
      Y_100 *= 3;

      if(X_100 < 10 && X_100 > -10) X_100 = 0;
      if(Y_100 < 10 && Y_100 > -10) Y_100 = 0;

      //motor
      A_v = (int)Y_100 * -1 + (int)X_100/2 + rotation;
      B_v = (int)Y_100 + (int)X_100/2 + rotation;
      C_v = (int)X_100 * -1 + rotation;

      if(A_out < A_v) A_out++;
      else A_out--;

      if(B_out < B_v) B_out++;
            else B_out--;

      if(C_out < C_v) C_out++;
            else C_out--;

      //sprintf(str,"%d %d %d %d %d %5d %5d",X_v,Y_v,A_v,B_v,C_v,X_encoder,Y_encoder);
      //sprintf(str,"%d %d",(int)X_100,(int)Y_100);
      sprintf(str, "Gyro / Acc: %6d %6d %6d %6d %6d %6d\r\n", mainBoard.mpu6050.readGyrX(), mainBoard.mpu6050.readGyrY(), mainBoard.mpu6050.readGyrZ(), mainBoard.mpu6050.readAccX(), mainBoard.mpu6050.readAccY(), mainBoard.mpu6050.readAccZ());
      debug << str << "\r\n";

      mainBoard.motorA.setOutput(A_out/500.0);
      mainBoard.motorB.setOutput(B_out/500.0);
      mainBoard.motorC.setOutput(C_out/500.0);


      MillisecondTimer::delay(2);

      X_encoder_old = X_encoder_now;
      Y_encoder_old = Y_encoder_now;
/*
      for(int i=0; i<100;i++){
	  mainBoard.motorA.setOutput((float)i/500.0);
	  mainBoard.motorB.setOutput((float)i/500.0);
	  mainBoard.motorC.setOutput((float)0/500.0);
	  MillisecondTimer::delay(50);
      }
      mainBoard.led.Off();
      for(int i=100; i>-100;i--){
	  mainBoard.motorA.setOutput((float)i/500.0);
	  mainBoard.motorB.setOutput((float)i/500.0);
	  mainBoard.motorC.setOutput((float)i/500.0);
	  //MillisecondTimer::delay(50);
      }
      */
  }

  while(1);

}

/* End Of File ---------------------------------------------------------------*/
