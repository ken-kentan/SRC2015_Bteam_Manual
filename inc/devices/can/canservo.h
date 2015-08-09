/*
 * canservo.h
 *
 *  Created on: 2014/10/05
 *      Author: ken_kentan
 */

#pragma once

#include "config/stm32plus.h"
#include "hardware/canroot.h"

namespace stm32plus{

  class CanServo {
  private:
    CanTxMsg TxMessage;
    CanRoot *can;
  public:
    CanServo(CanRoot *_can){
      can = _can;

      TxMessage.StdId = 0x170;
      TxMessage.ExtId = 0x01;
      TxMessage.RTR = CAN_RTR_DATA;
      TxMessage.IDE = CAN_ID_STD;
      TxMessage.DLC = 1;
      TxMessage.Data[0] = 0x00;
    }

    void Set(int ch){
      TxMessage.Data[0] |= 0x01 << ch;
    }

    void Reset(int ch){
      TxMessage.Data[0] &= ~(0x01 << ch);
    }

    void Update(){
      can->Send(&TxMessage);
    }

  };

}
