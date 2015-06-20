/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"

#include "config/stm32plus.h"
#include "config/gpio.h"
#include "config/timing.h"
#include "utils/debug.h"

#include "board/main_v3.h"

#include "utils/MadgwickAHRS.h"

using namespace stm32plus;

/* Defines -------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/


int main(void)
{

  //Initialise Systick
  MillisecondTimer::initialise();

  Nvic::initialise();

  MillisecondTimer::delay(100);

  MainV3 mainBoard;

  while(1){
	  debug << "test\r\n";

      MillisecondTimer::delay(50);
  }

}

/* End Of File ---------------------------------------------------------------*/
