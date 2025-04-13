#include "HexTransfer.h"


HexTransfer::HexTransfer()
{
  // Initialize the hex line 
  cur_hex_line_num = -1;
  cur_hex_line_chunk_count = -1;
  memset(cur_hex_line_data, PAD, sizeof(cur_hex_line_data));
  memset(chunks_received, false, sizeof(chunks_received));
}

HexTransfer::~HexTransfer()
{
}

void HexTransfer::process_can_msg(uint8_t (&buf)[8])
{
  // check if the first bit is 0
  if ((buf[0] & 0x01) == 0) {
    // Message is a TransferInitMsg
  }

  /// Message is a TransferChunkMsg
  // Unpack the message
  TransferChunkMsg msg = unpack_transfer_chunk_msg(buf);
  
  // Check if the message type is valid
  if (!process_transfer_chunk_msg(msg)) {
    // Handle error
    Serial.println("Error processing transfer chunk message!");
    return;
  }
  
  // Check if all chunks for the current line have been received
  if (all_chunks_received) {
    parse_and_validate_hex_line(cur_hex_line_data);
  }
}

TransferChunkMsg HexTransfer::unpack_transfer_chunk_msg(uint8_t (&buf)[8]) {
  // Initialize the TransferChunkMsg structure
  TransferChunkMsg m{};

  // Reconstruct the 64-bit integer from 8 Little Endian bytes
  uint64_t packed = 0;
  for (int i = 0; i < 8; i++) {
    packed |= (uint64_t)buf[i] << (8 * i);
  }

  // Extract each field from 'packed'
  m.msg_type      = (packed >> 0) & 0x1;
  m.line_num      = (packed >> 1) & 0x7FFF;
  m.chunk_num     = (packed >> 16) & 0x0F;
  m.total_chunks  = (packed >> 20) & 0x0F;

  // Then the next 40 bits contain data
  for (int i = 0; i < MAX_HEX_CHUNK_SIZE; i++) {
    m.hex_data[i] = (packed >> (24 + 8 * i)) & 0xFF;
  }

  // Return the unpacked message
  return m;
}

bool HexTransfer::process_transfer_chunk_msg(TransferChunkMsg &msg) {
  // Check if the line number matches the current line number
  if (msg.line_num != cur_hex_line_num) {
    // Line number does not match, handle error or reset
    Serial.println("Line number mismatch!");
    return false;
  }
  
  // Check if the chunk count matches the existing chunk count
  if (cur_hex_line_chunk_count == -1) {
    // First chunk, initialize the chunk count
    cur_hex_line_chunk_count = msg.total_chunks;
  }
  else if (msg.chunk_num != cur_hex_line_chunk_count) {
    // Chunk count does not match that of previous messages for this hex line
    Serial.println("Chunk number mismatch!");
    return false;
  }
  
  // Check if the chunk number is valid
  if (msg.chunk_num >= cur_hex_line_chunk_count) {
    // Invalid chunk number, handle error
    Serial.println("Invalid chunk number!");
    return false;
  }
  
  // Copy the 5 bytes of hex data into the current hex line data
  for (int i = 0; i < MAX_HEX_CHUNK_SIZE; i++) {
    cur_hex_line_data[msg.chunk_num * MAX_HEX_CHUNK_SIZE + i] = msg.hex_data[i];
  }
  
  // Log the received chunk in chunks_received
  chunks_received[msg.chunk_num] = true;
  
  // Update the all_chunks_received flag
  all_chunks_received = true;
  for (int i = 0; i < cur_hex_line_chunk_count; i++) {
    if (!chunks_received[i]) {
      all_chunks_received = false;
      break;
    }
  }
  
  // Return true
  return true;
}

ParsedHexLine HexTransfer::parse_and_validate_hex_line(const char (&buf)[MAX_HEX_LINE_SIZE])
{
  // Initialize the parsed hex line
  ParsedHexLine hex_line;
  
  // Find the length of the hex line. Unused bytes are filled with PAD (0xFF)
  size_t lineLen = 0;
  while (lineLen < MAX_HEX_LINE_SIZE && buf[lineLen] != PAD) {
    lineLen++;
  }
  
  // Check that the line is at least 11 bytes long (minimum length for a valid hex line)
  if (lineLen < 11) {
    hex_line.valid = false;
    return hex_line;
  }
  
  // Check that the first byte is a colon
  if (buf[0] != ':') {
    hex_line.valid = false;
    return hex_line;
  }

  // We expect the layout:
  //   : [byte_count:2 hex] [address:4 hex] [rec_type:2 hex] [data:2 * byte_count hex] [checksum:2 hex]
  // Read more about the Intel HEX format here: https://en.wikipedia.org/wiki/Intel_HEX  
  
  // Create a pointer to itorate through the string
  const char *ptr;
  // Set the pointer to the first byte after the colon
  ptr = buf + 1;
  
  // Parse the byte count
  if (!sscanf (ptr, "%02x", &hex_line.byte_count)) {
    hex_line.valid = false;
    return hex_line;
  }
  // Move the pointer bast the byte count value
  ptr += 2;
  
  // Check if the byte count is 16 or less
  // NOTE: The technical limit is 255, but the teensy3.5 only 
  //       uses 16 bytes chunks in the data records of its hex files
  if (hex_line.byte_count > 16) {
    hex_line.valid = false;
    return hex_line;
  }
  
  // Check if the byte count is valid. (I.E. the line is as long as it says)
  // We add 11 because there is always a colon(1), byte count(2), address(4), record type(2), and checksum(2)
  // We multiply the byte count by 2 because each byte is represented by 2 hex digits written in ascii
  if (lineLen != (11 + hex_line.byte_count * 2)) {
    hex_line.valid = false;
    return hex_line;
  }
  
  // Parse the address
  if (!sscanf (ptr, "%4x", &hex_line.address)) {
    hex_line.valid = false;
    return hex_line;
  }
  // Move the pointer past the address value
  ptr += 4;
  
  // Parse the record type
  if (!sscanf (ptr, "%02x", &hex_line.record_type)) {
    hex_line.valid = false;
    return hex_line;
  }
  // Move the pointer past the record type value
  ptr += 2;
  
  // Parse the data bytes
  for (size_t i = 0; i < hex_line.byte_count; i++) {
    if (!sscanf (ptr, "%02x", &hex_line.data[i])) {
      hex_line.valid = false;
      return hex_line;
    }
    // Move the pointer past the data value
    ptr += 2;
  }
  
  hex_line.address = ((buf[3] - '0') * 16 + (buf[4] - '0')) << 8;
  
  // Parse the record type
  return hex_line;
}