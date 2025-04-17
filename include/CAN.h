/**
   CAN.h - Header for CAN.cpp.
*/
#ifndef CAN_h
#define CAN_h

#include "Arduino.h"
#include <i2c_t3.h>

#include "FlexCAN.h"
#include "HexTransfer.h"

namespace CAN {
  union FloatToBytes {
    float val;
    uint8_t bytes[4];
  };
  union Int32ToBytes {
    int32_t val;
    uint8_t bytes[4];
  };

  
  void init();
  void print_hex_payload_msg(uint8_t (&buf)[8]);
  void handleInbox();
  void wipeMessage();
  boolean write(CAN_message_t msg);
  void _printCAN(CAN_message_t txmsg);
  void write(uint8_t deviceID, uint8_t commandID, uint8_t payloadLength, uint8_t buffer[]);
  void write(uint8_t deviceID, uint8_t commandID, int32_t payload);
  void write(uint8_t deviceID, uint8_t commandID, uint8_t payload);
  void write(uint8_t deviceID, uint8_t commandID, int payload);
  void write(uint8_t deviceID, uint8_t commandID, float payload);
  float readFloat(CAN_message_t rxmsg);
  int32_t readInt32(CAN_message_t rxmsg);
}

#endif
