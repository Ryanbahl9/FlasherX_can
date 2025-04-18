/**
 * CAN.cpp - Helper for constructing and sending CAN bus messages.
 */
#include "CAN.h"

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
      HexTransfer::handle_can_msg(rxmsg.buf);
    }
    else {
      Serial.print("CAN message from device: ");
      Serial.println(deviceID);
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

