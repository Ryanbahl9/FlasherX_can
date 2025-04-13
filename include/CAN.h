/**
   CAN.h - Header for CAN.cpp.
*/
#ifndef CAN_h
#define CAN_h

#include "Arduino.h"
#include <i2c_t3.h>

#include "FlexCAN.h"

struct CanHexPayloadMsg {
  // Total of 64 bits (8 bytes):
  uint64_t msg_type     : 1;   // Bit 1: message type (1 bit)
  uint64_t line_number  : 15;  // Bits 2–16: the line number (15 bits)
  uint64_t chunk_num    : 4;   // Bits 17–20: current chunk number (4 bits)
  uint64_t total_chunks : 4;   // Bits 21–24: total number of chunks the hex line has been split into (4 bits)
  uint64_t hex_data     : 40;  // Bits 25–64: actual hex data (40 bits)
};

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
