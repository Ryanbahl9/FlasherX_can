#include "HexTransfer.h"

namespace HexTransfer
{
  // --------------------------------------------------------------------------
  // Flash Buffer Variables
  // --------------------------------------------------------------------------
  // Info about the buffer in flash space. This is set externally by the

  uint32_t flash_buffer_addr; // Address of the buffer in flash space
  
  uint32_t flash_buffer_size; // Size of the buffer in flash space
  
  bool flash_buffer_initialized; // Flag to indicate if the buffer has been initialized

  // --------------------------------------------------------------------------
  // Hex File Info Variables
  // --------------------------------------------------------------------------
  // These variables are used to store information about the hex file being
  // transferred.
  //
  // The base_address, start_address, and eof_received fields represent standard
  // Intel HEX file information. For details, see:
  //   https://en.wikipedia.org/wiki/Intel_HEX
  //
  // The min_address, max_address, total_lines, and file_checksum fields are used for
  // validating the hex file and ensuring that the entire file has been
  // received correctly.
  // 
  // Note:
  //   - Logic for Hex records 02 (Extended Segment Address) and 03 (Start 
  //     Segment Address) are present future compatibility with other 
  //     platforms, but are not used because no Teensy board employs segmented
  //     addressing.
  
  // Base address to be added to each hex line address.
  // Set by hex records: 02 (Extended Segment Address)
  //                     04 (Extended Linear Address)
  uint32_t base_address;

  // Starting address of the hex file in teensy's flash
  // Set by hex records: 03 (Start Segment Address)
  //                     05 (Start Linear Address)
  // Unused for teensyduinos, they always set the start address to 0x0000. Leaving
  // this here for future compatibility with other platforms.
  uint32_t start_address; 

  // Minimum address in the hex file in teensy's flash.
  // Calculated while processing hex data records.
  uint32_t min_address;   

  // Maximum address in the hex file in teensy's flash
  // Calculated while processing hex data records.
  uint32_t max_address;   
                          
  // Flag to indicate if EOF has been reached and eof record has been received
  bool eof_received;

  // Total number of line we expect to receive in the hex file
  size_t total_lines;     
 
  // Checksum of the entire hex file in CRC32 format
  uint32_t received_file_checksum;  

  // --------------------------------------------------------------------------
  // Current Hex Line Variables
  // --------------------------------------------------------------------------
  // These variables are used to store information about the current hex line
  // being received.
  // This buffer will be eventually parsed into a ParsedHexLine struct.
  
  // Current hex line number being received. 0 indexed.
  size_t hex_line_num;                  

  // Number of segments the current hex line has been split into
  int hex_line_segment_count;
  
  // Array to keep track of which segments have been received for the current
  // hex line
  bool* hex_line_segments_received;     

  // The buffer the segments are copied into
  char hex_line_buf[MAX_HEX_LINE_SIZE]; 
  
  // --------------------------------------------------------------------------
  // Hex Transfer State Variables
  // --------------------------------------------------------------------------
  // These variables are used to keep track of the state of the hex transfer
  
  // Flag to indicate if a transfer init message was received in the last cycle.
  // This is used to determine if we should send an acknowledgment this cycle.
  bool new_transfer_init_msg_received;  
  
  // Flag to indicate if the transfer init message was received and valid
  bool transfer_init_msg_error;     

  // Flag to indicate if a transfer is in progress. Set when a Transfer Init
  // message is received and cleared when the transfer is complete or aborted.
  bool transfer_in_progress;        

  // Flag to indicate if the file transfer is complete. Set when all lines have
  // been received and the checksum is valid.
  bool file_transfer_complete;      
  
  // Checksum of the hex file being received. This is calculated by adding the 
  // checksum of each hex line as it is received.
  uint32_t computed_file_checksum;  

  // CRC32 object for calculating the checksum of the hex file
  FastCRC32 CRC32;
  
  // --------------------------------------------------------------------------
  // Timeout Variables
  // --------------------------------------------------------------------------
  // These variables are used to keep track of the timeouts for the hex transfer
  // and the inactivity timeout.
  
  uint32_t last_successful_can_msg_ts;

} // namespace HexTransfer



// --------------------------------------------------------------------------
// Main Functions
// --------------------------------------------------------------------------
void HexTransfer::init(){ 
  // Initialize the hex file info variables
  clear_transfer_state();
}

void HexTransfer::update() {
  // Return if no transfer is in progress
  if (!transfer_in_progress) return;
  
  ResponseCode res = ResponseCode::NONE;
  
  // Check if the transfer has timed out
  if (has_transfer_timed_out()) {
    res = ResponseCode::ERROR;
    abort_transfer();
  }
  // Check if the segment has timed out
  else if (has_segment_timed_out()) {
    // Request the a line without incrementing the line number
    // PC will resend the same line
    res = ResponseCode::SEND_LINE;
  }
  // Check if a new transfer init message has been received
  else if (new_transfer_init_msg_received ) {
    res = transfer_init_msg_error
            ? ResponseCode::ERROR
            : ResponseCode::SEND_LINE;
  }
  // Handle the received hex line if all segments have been received
  else if (are_all_segments_received()) {
    res = handle_received_hex_line();
  }
  // Check if the EOF record has been received
  else if (eof_received) {
    if (!is_file_checksum_valid()) {
      res = ResponseCode::ERROR;
      abort_transfer();
    }
    else {
      res = ResponseCode::TRANSFER_COMPLETE;
      transfer_in_progress = false;
      file_transfer_complete = true;
    }
  }
  
  // Send the response
  send_response(res);
}

// --------------------------------------------------------------------------
// Can Bus Message Handlers
// --------------------------------------------------------------------------

void HexTransfer::handle_can_msg(uint8_t (&buf)[8])
{ 
  // Check if the message is a TransferInitMsg or a TransferSegmentMsg
  if ((buf[0] & 0x01) == 0) {
    // Message is a TransferInitMsg
    // Unpack the message
    TransferInitMsg msg = unpack_transfer_init_msg(buf);

    #if DEBUG
    print_transfer_init_msg(msg);
    #endif

    // Process and Report if the message is invalid
    if (!process_transfer_init_msg(msg)) {
      #if DEBUG
      Serial.println("Error processing transfer init message!");
      #endif
      return;
    }
  }
  else if (transfer_in_progress) {
    // Message is a TransferSegmentMsg
    // Unpack the message
    TransferSegmentMsg msg = unpack_transfer_segment_msg(buf);
    
    #if DEBUG
    print_transfer_segment_msg(msg);
    #endif
    
    // Process and Report if the message is invalid
    if (!process_transfer_segment_msg(msg)) {
      #if DEBUG
      Serial.println("Error processing transfer segment message!");
      #endif
      return;
    }
  }
  
  // Update the last successful CAN message timestamp
  last_successful_can_msg_ts = millis();
}

HexTransfer::TransferInitMsg HexTransfer::unpack_transfer_init_msg(uint8_t (&buf)[8]) {
  // Initialize the TransferInitMsg structure
  TransferInitMsg m{};

  // Reconstruct the 64-bit integer from 8 Little Endian bytes
  uint64_t packed = 0;
  for (int i = 0; i < 8; i++) {
    // Shift the byte into the correct position in the 'packed' integer
    packed |= (uint64_t)buf[i] << (8 * i);
  }

  // Extract each field from 'packed'
  m.msg_type           = (packed >> 0) & 0x1;    // 0x1 = 2^1 - 1      (1 bit mask)
  m.line_count         = (packed >> 1) & 0x7FFF; // 0x7FFF = 2^15 - 1 (15 bit mask)
  m.file_checksum      = (packed >> 16) & 0xFFFFFFFF; // 0xFFFFFFFF = 2^32 - 1 (32 bit mask)
  m.init_msg_checksum   = (packed >> 48) & 0xFFFF; // 0xFFFF = 2^16 - 1 (16 bit mask)

  // Calculate the checksum of the message
  const uint8_t* data = reinterpret_cast<const uint8_t*>(&m);
  m.calculated_msg_checksum = CRC32.crc32(data, (unsigned int)(48));
  // Return the unpacked message
  return m;
}

HexTransfer::TransferSegmentMsg HexTransfer::unpack_transfer_segment_msg(uint8_t (&buf)[8]) {
  // Initialize the TransferSegmentMsg structure
  TransferSegmentMsg m{};

  // Reconstruct the 64-bit integer from 8 Little Endian bytes
  uint64_t packed = 0;
  for (int i = 0; i < 8; i++) {
    // Shift the byte into the correct position in the 'packed' integer
    packed |= (uint64_t)buf[i] << (8 * i);
  }

  // Extract each field from 'packed'
  m.msg_type      = (packed >> 0) & 0x1;    // 0x1 = 2^1 - 1      (1 bit mask)
  m.line_num      = (packed >> 1) & 0x7FFF; // 0x7FFF = 2^15 - 1  (15 bit mask)
  m.segment_num     = (packed >> 16) & 0x0F;  // 0x0F = 2^4 - 1     (4 bit mask)
  m.total_segments  = (packed >> 20) & 0x0F;  // 0x0F = 2^4 - 1     (4 bit mask)

  // Then the next 40 bits contain data
  for (int i = 0; i < MAX_HEX_CHUNK_SIZE; i++) {
      m.hex_data[i] = static_cast<char>((packed >> (24 + 8 * i)) & 0xFF); // 0xFF = 2^8 - 1 (8 bit mask)
    }

  // Return the unpacked message
  return m;
}

bool HexTransfer::process_transfer_init_msg(TransferInitMsg &msg) {
  // Check if the message type is valid
  if (msg.msg_type != 0) {
    return false;
  }
  
  // Log the received message
  new_transfer_init_msg_received = true;

  // Check if the checksum is valid
  if (msg.init_msg_checksum != msg.calculated_msg_checksum) {
    // Checksum error, return false
    transfer_init_msg_error = true;
    return false;
  }
  
  // Log the successful message
  transfer_init_msg_error = false;
  
  // Abort any previous transfers if any
  abort_transfer();

  // Set the transfer in progress flag
  transfer_in_progress = true;
  
  // Set the file checksum
  received_file_checksum = msg.file_checksum;
  
  // Set the line count
  total_lines = msg.line_count;
  
  // Return success
  return true;
}

bool HexTransfer::process_transfer_segment_msg(TransferSegmentMsg &msg) {
  // Check if the line number matches the current line number
  if (msg.line_num != hex_line_num) {
    // Line number does not match, handle error or reset
    Serial.print("Line number mismatch! ");
    Serial.print(msg.line_num);
    Serial.print(" != ");
    Serial.println(hex_line_num);
    return false;
  }
  
  // Check if the segment count matches the existing segment count
  if (hex_line_segment_count == -1) {
    // First segment, initialize the segment count and hex_line_segments_received list
    hex_line_segment_count = msg.total_segments;
    hex_line_segments_received = new bool[hex_line_segment_count];
    for (int i = 0; i < hex_line_segment_count; i++) {
      hex_line_segments_received[i] = false;
    }
  }
  else if (msg.total_segments != hex_line_segment_count) {
    // Segment count does not match that of previous messages for this hex line
    Serial.print("Segment number mismatch!");
    Serial.print(msg.segment_num);
    Serial.print(" != ");
    Serial.println(hex_line_segment_count);
    return false;
  }
  
  // Check if the segment number is valid
  if (msg.segment_num >= hex_line_segment_count) {
    // Invalid segment number, handle error
    Serial.print("Invalid segment number! ");
    Serial.print(msg.segment_num);
    Serial.print(" >= ");
    Serial.println(hex_line_segment_count);
    return false;
  }
  
  // Copy the 5 bytes of hex data into the current hex line data
  for (int i = 0; i < MAX_HEX_CHUNK_SIZE; i++) {
    hex_line_buf[msg.segment_num * MAX_HEX_CHUNK_SIZE + i] = msg.hex_data[i];
  }
  
  // Mark the segment as received
  hex_line_segments_received[msg.segment_num] = true;
  
  // Return true
  return true;
}

bool HexTransfer::send_response(ResponseCode res, ErrorCode err) {
  // Create a response message
  uint8_t buf[8] = {0};
  // Pack the response message
  if (!pack_response(res, buf)) {
    // Error packing the response message
    return false;
  }
  /// TODO: Send the response message over CAN bus
  return true;
}

bool HexTransfer::pack_response(ResponseCode res, uint8_t (&buf)[8]) {
  /// TODO: Pack the response message into the buffer
  
  // Return success
  return true;
}

// --------------------------------------------------------------------------
// Hex Line Processing Functions
// --------------------------------------------------------------------------

HexTransfer::ResponseCode HexTransfer::handle_received_hex_line() {
  // All segments have been received, parse and validate the hex line
  ParsedHexLine hex_line = parse_and_validate_hex_line(hex_line_buf);
  
  // Check if the hex line is valid
  if (!hex_line.valid) {
    reset_cur_hex_line_buff();
    // Send a line request with incremented line number
    // PC will resend the same line
    return ResponseCode::SEND_LINE;
  }

  // Process the hex line
  if (!process_hex_line(hex_line)) {
    reset_cur_hex_line_buff();
    // Send a line request with incremented line number
    // PC will resend the same line
    return ResponseCode::SEND_LINE;
  }
  
  // Add the hex line to the checksum
  add_hex_line_to_checksum();
  
  // Increment the line number
  hex_line_num++;
  
  // Clear the hex line buffer
  reset_cur_hex_line_buff();
  
  // Return success
  return ResponseCode::SEND_LINE;
}

HexTransfer::ParsedHexLine HexTransfer::parse_and_validate_hex_line(const char (&buf)[MAX_HEX_LINE_SIZE])
{
  // Checks Done for Line Validation:
  // 1. Line is at least 11 bytes long
  // 2. Line starts with a colon
  // 3. Byte count is less than or equal to 16 
  //    (Format allows 255, but teensy3.5 only uses 16 bytes segments)
  // 4. Line length matches the byte count
  //    (Line length = 11 + byte_count * 2)
  // 5. Record type is valid (Must be between 0 and 5)
  // 6. Checksum is valid

  // Data Parsed from the hex line
  // Parse 1. byte count
  // Parse 2. address
  // Parse 3. record type
  // Parse 4. data bytes
  // Parse 5. checksum
  
  // Initialize the parsed hex line
  ParsedHexLine hex_line;
  hex_line.valid = true;
  unsigned int checksum = 0;
  
  // Find the length of the hex line. Unused bytes are filled with PAD (0xFF)
  size_t lineLen = 0;
  while (lineLen < MAX_HEX_LINE_SIZE && buf[lineLen] != PAD) {
    lineLen++;
  }
  
  // Check 1: Line is at least 11 bytes long
  if (lineLen < 11) {
    #if DEBUG
    Serial.print("Error: Hex line length is less than 11 bytes! Line length: ");
    Serial.println(lineLen);
    #endif
    
    hex_line.valid = false;
    return hex_line;
  }
  
  // Check 2: Line starts with a colon
  if (buf[0] != ':') {
    #if DEBUG
    Serial.println("Error: Hex line does not start with a colon!");
    #endif
    
    hex_line.valid = false;
    return hex_line;
  }

  // We expect the layout to be:
  //   : [byte_count:2 hex] [address:4 hex] [rec_type:2 hex] [data:2 * byte_count hex] [checksum:2 hex]
  // Read more about the Intel HEX format here: https://en.wikipedia.org/wiki/Intel_HEX  
  
  // Create a pointer to itorate through the string
  const char *ptr;
  // Set the pointer to the first byte after the colon
  ptr = buf + 1;
  
  // Parse 1: Byte count
  if (!sscanf (ptr, "%02x", &hex_line.byte_count)) {
    #if DEBUG
    Serial.println("Error: Unable to parse byte count!");
    #endif
    
    hex_line.valid = false;
    return hex_line;
  }
  checksum += hex_line.byte_count;
  ptr += 2;
  
  // Check 3: Check if the byte count is valid.
  // NOTE: The technical limit is 255, but the teensy3.5 only 
  //       uses 16 bytes segments in the data records of its hex files
  if (hex_line.byte_count > 16) {
    #if DEBUG
    Serial.println("Error: Byte count is greater than 16!");
    #endif
    
    hex_line.valid = false;
    return hex_line;
  }
  
  // Check 4: Check if the line length matches the byte count
  // We add 11 because there is always a colon(1), byte count(2), address(4), record type(2), and checksum(2)
  // We multiply the byte count by 2 because each byte is represented by 2 hex digits written in ascii
  if (lineLen != (11 + hex_line.byte_count * 2)) {
    #if DEBUG
    Serial.print("Error: Line length does not match byte count! Line length: ");
    Serial.println(lineLen);
    #endif
    
    hex_line.valid = false;
    return hex_line;
  }
  
  // Parse 2: Address
  if (!sscanf (ptr, "%4x", &hex_line.address)) {
    #if DEBUG
    Serial.println("Error: Unable to parse address!");
    #endif
    
    hex_line.valid = false;
    return hex_line;
  }
  checksum += (hex_line.address >> 8) & 0xFF;
  ptr += 4;
  
  // Parse 3: Record type
  if (!sscanf (ptr, "%02x", &hex_line.record_type)) {
    #if DEBUG
    Serial.println("Error: Unable to parse record type!");
    #endif
    
    hex_line.valid = false;
    return hex_line;
  }
  ptr += 2;

  // Check 5: Check if the record type is valid
  if (hex_line.record_type > 5) {
    #if DEBUG
    Serial.println("Error: Record type is invalid!");
    #endif
    
    hex_line.valid = false;
    return hex_line;
  }

  // Parse the data bytes
  // Note: past this point the data is stored as raw bytes
  //       not as hex represented in ascii like before.
  for (size_t i = 0; i < hex_line.byte_count; i++) {
    if (!sscanf (ptr, "%02x", &hex_line.data[i])) {
      #if DEBUG
      Serial.println("Error: Unable to parse data bytes!");
      #endif
      
      hex_line.valid = false;
      return hex_line;
    }
    // Move the pointer past the data value
    ptr += 2;
  }
  
  // Parse the checksum
  if (!sscanf (ptr, "%02x", &hex_line.checksum)) {
    #if DEBUG
    Serial.println("Error: Unable to parse checksum!");
    #endif
    
    hex_line.valid = false;
    return hex_line;
  }  
  
  // Return the parsed hex line
  return hex_line;
}

bool HexTransfer::process_hex_line(ParsedHexLine &hex_line) {
  // Create a switch statement to handle the different record types
  switch (hex_line.record_type) {
    case 0:
      return process_hex_data_record(hex_line);
    case 1:
      return process_hex_eof_record(hex_line);
    case 2:
      return process_hex_extended_segment_address_record(hex_line);
    case 3:
      return process_hex_start_segment_address_record(hex_line);
    case 4:
      return process_hex_extended_linear_address_record(hex_line);
    case 5:
      return process_hex_start_linear_address_record(hex_line);
    default:
      // Invalid record type, handle error
      return false;
  }
}

bool HexTransfer::process_hex_data_record(ParsedHexLine &hex_line) {
  // Check if the record type is Data
  if (hex_line.record_type != 0) {
    #if DEBUG
    Serial.println("Error: Record type is not Data!");
    #endif
    
    return false;
  }
  
  // Update the min and max addresses
  if (base_address + hex_line.address + hex_line.byte_count > max_address) {
    max_address = base_address + hex_line.address + hex_line.byte_count;
  }
  if (base_address + hex_line.address < min_address) {
    min_address = base_address + hex_line.address;
  }
  
  // Check if the address is too large
  if (max_address > (FLASH_BASE_ADDR + flash_buffer_size)) {
    #if DEBUG
    Serial.println("Error: Address is too large!");
    #endif
    
    return false;
  }
  
  // #if not DRYRUN
  #if not DRYRUN
  
  // Calculate the address in the flash buffer we will copy the data to
  uint32_t addr = flash_buffer_addr + base_address + hex_line.address - FLASH_BASE_ADDR;
  
  if (IN_FLASH(flash_buffer_addr)) {
    char *bytePtr = reinterpret_cast<char*>(hex_line.data);
    int error = flash_write_block( addr, bytePtr, (uint32_t)hex_line.byte_count );
    if (error) {
      #if DEBUG
      Serial.printf( "abort - error %02X in flash_write_block()\n", error );
      #endif
      
      return false;
    }
  }
  else if (!IN_FLASH(flash_buffer_addr)) {
    // This is to support RAM buffer transfers, not available on Teensy 3.5
    memcpy(reinterpret_cast<void*>(addr), hex_line.data, hex_line.byte_count);
  }
  #endif
  return true;
}

bool HexTransfer::process_hex_eof_record(ParsedHexLine &hex_line) {
  // Check if the record type is EOF
  if (hex_line.record_type != 1) {
    #if DEBUG
    Serial.println("Error: Record type is not EOF!");
    #endif
    
    return false;
  }
  
  // Check if this is the last line
  if (hex_line_num != total_lines - 1) {
    #if DEBUG
    Serial.println("Error: EOF record is not the last line!");
    #endif
    
    // Not the last line, handle error
    return false;
  }

  // Set the EOF flag
  eof_received = true;

  // Return success
  return true;
}

bool HexTransfer::process_hex_extended_segment_address_record(ParsedHexLine &hex_line) {
  // Check if the record type is Extended Segment Address
  if (hex_line.record_type != 2) {
    #if DEBUG
    Serial.println("Error: Record type is not Extended Segment Address!");
    #endif
    
    return false;
  }

  // Set the base address for the hex file
  base_address = ((hex_line.data[0] << 8) | hex_line.data[1]) << 4;

  // Return success
  return true;
}

bool HexTransfer::process_hex_start_segment_address_record(ParsedHexLine &hex_line) {
  // Check if the record type is Start Segment Address
  if (hex_line.record_type != 3) {
    #if DEBUG
    Serial.println("Error: Record type is not Start Segment Address!");
    #endif
    
    return false;
  }

  // This record type is not used in this implementation
  // This is used to set the starting execution address for the program
  // Teensyduinos always sets the start address to 0x0000
  #if DEBUG
  Serial.println("Warning: Start Segment Address record is not implemented!");
  #endif
  
  // Return success
  return true;
}

bool HexTransfer::process_hex_extended_linear_address_record(ParsedHexLine &hex_line) {
  // Check if the record type is Extended Linear Address
  if (hex_line.record_type != 4) {
    #if DEBUG
    Serial.println("Error: Record type is not Extended Linear Address!");
    #endif
    
    return false;
  }

  // Set the base address for the hex file
  base_address = ((hex_line.data[0] << 8) | hex_line.data[1]) << 16;

  // Return success
  return true;
}

bool HexTransfer::process_hex_start_linear_address_record(ParsedHexLine &hex_line) {
  // Check if the record type is Start Linear Address
  if (hex_line.record_type != 5) {
    #if DEBUG
    Serial.println("Error: Record type is not Start Linear Address!");
    #endif
    
    return false;
  }

  // This record type is not used in this implementation
  // This is used to set the starting execution address for the program
  // Teensyduinos always sets the start address to 0x0000

  // Return success
  return true;
}


// --------------------------------------------------------------------------
// Helper Functions
// --------------------------------------------------------------------------

bool HexTransfer::are_all_segments_received() {
  // Check if all segments have been received
  for (int i = 0; i < hex_line_segment_count; i++) {
    if (!hex_line_segments_received[i]) {
      return false;
    }
  }
  return true;
}

void HexTransfer::add_hex_line_to_checksum() {
  // Get the length of the hex line without the padding
  uint16_t len = 0;
  while (len < MAX_HEX_LINE_SIZE && hex_line_buf[len] != PAD) {
    len++;
  }
  
  const uint8_t* data = reinterpret_cast<const uint8_t*>(hex_line_buf);

  // Add the hex line to the checksum
  computed_file_checksum = CRC32.crc32_upd(data, len);
}

bool HexTransfer::is_file_checksum_valid() {
  // Check if the computed file checksum matches the expected checksum
  if (computed_file_checksum != received_file_checksum) {
    return false;
  }
  return true;
}

void HexTransfer::clear_transfer_state() {
  base_address = 0;
  start_address = 0;
  min_address = 0xFFFFFFFF;
  max_address = 0;
  eof_received = false;
  total_lines = 0;
  received_file_checksum = 0;
  hex_line_num = 0;
  new_transfer_init_msg_received = false;
  transfer_init_msg_error = false;
  transfer_in_progress = false;
  file_transfer_complete = false;
  computed_file_checksum = CRC32.crc32((uint8_t*)"", 0); // Initialize to 0
  
  reset_cur_hex_line_buff();
}

void HexTransfer::reset_cur_hex_line_buff() {
  hex_line_segment_count = -1;
  if (hex_line_segments_received != nullptr) {
    delete[] hex_line_segments_received;
  }
  hex_line_segments_received = nullptr;
  memset(hex_line_buf, PAD, sizeof(hex_line_buf));
}

void HexTransfer::abort_transfer() {
  // Clear the transfer state
  clear_transfer_state();
  
  #if DEBUG
  Serial.println("Transfer aborted!");
  #endif
}

bool HexTransfer::is_transfer_in_progress() {
  // Check if a transfer is in progress
  return transfer_in_progress;
}

bool HexTransfer::is_file_transfer_complete() {
  // Check if the file transfer is complete
  return file_transfer_complete;
}

bool HexTransfer::has_segment_timed_out() {
  // Check if the segment has timed out
  return (millis() - last_successful_can_msg_ts) > HEX_LINE_TIMEOUT_LEN;
}

bool HexTransfer::has_transfer_timed_out() {
  // Check if the transfer has timed out
  return (millis() - last_successful_can_msg_ts) > INACTIVITY_TIMEOUT_LEN;
}

void HexTransfer::print_transfer_segment_msg(TransferSegmentMsg &msg) {
  // Print the transfer segment message
  Serial.print(msg.msg_type);
  Serial.print(" ");
  Serial.print(msg.line_num);
  Serial.print(" ");
  Serial.print(msg.segment_num);
  Serial.print(" ");
  Serial.print(msg.total_segments);
  Serial.print(" ");
  for (int j = 0; j < MAX_HEX_CHUNK_SIZE; j++) {
    if (msg.hex_data[j] != PAD) {
      Serial.print(msg.hex_data[j]);
    }
    else {
      Serial.print(".");
    }
  }
  Serial.println();
}

void HexTransfer::print_transfer_init_msg(TransferInitMsg &msg) {
  // Print the transfer segment message
  Serial.print(msg.msg_type);
  Serial.print(" ");
  Serial.print(msg.line_count);
  Serial.print(" ");
  Serial.print(msg.file_checksum);
  Serial.print(" ");
  Serial.print(msg.init_msg_checksum);
  Serial.println();
}