/**
 * CAN.cpp - Helper for constructing and sending CAN bus messages.
 */
#include "CAN.h"

#define PAD 0xFF 
#define HEX_DATA_CHUNK_SIZE 5         // 5 bytes per segment

FlexCAN CANbus(500000);

namespace CAN {
  static CAN_message_t rxmsg;  // Used to store incoming messages
}

void CAN::init() {
  CANbus.begin();
}

void CAN::handleInbox() {
  while (CANbus.read(rxmsg)) {
    uint8_t deviceID = (uint8_t) (rxmsg.id & 0xFFu);
    // uint8_t msgID = (uint8_t) (rxmsg.id / 256);
    
    if (deviceID == 0x0) {
      HexTransfer::process_can_msg(rxmsg.buf);
    }
    
    CAN::wipeMessage();
  }
}

void CAN::wipeMessage() {
  rxmsg.id = 0;
  rxmsg.ext = 0;
  rxmsg.len = 0;
  rxmsg.timeout = 0;
  for (int i = 0; i < 8; i++) {
    rxmsg.buf[i] = 0;
  }
}

boolean CAN::write(CAN_message_t msg) {
  if (CANbus.write(msg))
    return true;
  else
    return false;
}

void CAN::write(uint8_t deviceID, uint8_t commandID, uint8_t payloadLength, uint8_t buffer[]) {
  uint16_t fullID = (uint16_t) deviceID + (((uint16_t) commandID) << 8);
  uint8_t ext = 1;  // Extend ID by 1 byte
  uint16_t timeout = 0;
  CAN_message_t txmsg = {fullID, ext, payloadLength, timeout};
  memcpy(txmsg.buf, buffer, payloadLength);
  CAN::write(txmsg);
//  CAN::_printCAN(txmsg);
}

void CAN::write(uint8_t deviceID, uint8_t commandID, int32_t payload) {
  Int32ToBytes v = {payload};
  CAN::write(deviceID, commandID, 4, v.bytes);
}

void CAN::write(uint8_t deviceID, uint8_t commandID, uint8_t payload) {
  uint8_t buffer[1] = {payload};
  CAN::write(deviceID, commandID, 1, buffer);
}

void CAN::write(uint8_t deviceID, uint8_t commandID, int payload) {
  CAN::write(deviceID, commandID, (uint8_t) (payload & 0xFFu));
}

void CAN::write(uint8_t deviceID, uint8_t commandID, float payload) {
  FloatToBytes v = {payload};
  CAN::write(deviceID, commandID, 4, v.bytes);
}

void CAN::_printCAN(CAN_message_t msg) {
  Serial.print("NEW MESSAGE (id): "); Serial.println(msg.id);
  Serial.print("devid: "); Serial.println(msg.id%256);
  Serial.print("msgid: "); Serial.println(msg.id/256);
  Serial.print("ext: "); Serial.println(msg.ext);
  Serial.print("len: "); Serial.println(msg.len);
  Serial.print("timeout: "); Serial.println(msg.timeout);
  Serial.print("buf: ");
  for (uint8_t i = 0; i < msg.len; i++) {
    Serial.print(msg.buf[i]); Serial.print(" ");
  }
  Serial.println();
  if (msg.len == 4) {
    FloatToBytes conv;
    memcpy(conv.bytes, msg.buf, 4);
    Serial.print("  if float: "); Serial.println(conv.val);
  }
}

/**
 * @brief Read first four bytes of rxmsg.buf as a float (using a union).
 * 
 * @param rxmsg Message with float payload in bytes.
 * @return float 
 */
float CAN::readFloat(CAN_message_t rxmsg) {
  CAN::FloatToBytes ftb;
  memcpy(ftb.bytes, rxmsg.buf, 4);
  return ftb.val;
}

/**
 * @brief Read first four bytes of rxmsg.buf as an int32 (using a union).
 * 
 * @param rxmsg Message with int32_t payload in bytes.
 * @return int32_t
 */
int32_t CAN::readInt32(CAN_message_t rxmsg) {
  CAN::Int32ToBytes itb;
  memcpy(itb.bytes, rxmsg.buf, 4);
  return itb.val;
}
