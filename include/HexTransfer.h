#ifndef HEXTRANSFER_H
#define HEXTRANSFER_H

#include "Arduino.h"
#include <FastCRC.h>
extern "C" {
  #include "FlashTxx.h"		// TLC/T3x/T4x/TMM flash primitives
}

#define DEBUG 1
#define DRYRUN 1
namespace HexTransfer {
  

  #define MAX_HEX_LINE_SIZE 45      // Max size of hex line data, in bytes
  #define MAX_HEX_CHUNK_SIZE 5      // Max size of hex data in a segment, in bytes
  #define MAX_CHUNKS_PER_HEX_LINE 9 // 45/5 = 9
  #define PAD 0xFF 
  
  #define HEX_LINE_TIMEOUT_LEN 5000     // Timeout for receiving hex line segments, in ms
  #define INACTIVITY_TIMEOUT_LEN 15000  // Timeout for inactivity, in ms

  #define PC_CAN_DEVICE_ID 0x0 // PC CAN ID
  #define PC_CAN_COMMAND_ID 0x0 // PC CAN message ID
  // -----------------------------------------------------------------
  // Hex Transfer Enums
  // -----------------------------------------------------------------
  enum class ResponseCode {
    NONE = 0,
    SEND_LINE = 1, // 
    TRANSFER_COMPLETE = 2,
    ERROR = 3,
  };
  
  enum class ErrorCode {
    NONE = 0,
    TRANSFER_NOT_IN_PROGRESS,
    TRANSFER_INIT_CHECKSUM_ERROR,
    TRANSFER_RETRY_LIMIT_EXCEEDED,
    INACTIVITY_TIMEOUT,
    FILE_CHECKSUM_ERROR
  };
  
  // ----------------------------------------------------------------------------
  // Hex Transfer Structs
  // ----------------------------------------------------------------------------

  // TransferInitMsg is meant to be packed into 8 bytes for CAN transfer.
  // The bit numbers on the right describe how it is packed into the 8 bytes
  // TransferInitMsg is the first message sent to initialize the transfer
  struct TransferInitMsg {
    bool msg_type;              // Bit 0: message type (1 bit)
    uint16_t line_count;        // Bits 1–15: total number of lines in the hex file (15 bits)
    uint32_t file_checksum;     // Bits 16–47: total file checksum (32 bits)
    uint16_t init_msg_checksum; // Bits 48–63: checksum of the init message (16 bits)
    uint16_t calculated_msg_checksum; // Not included in the packed message, but used for validation
  };

  // TransferSegmentMsg holds a single segment of a hex line and the information about it.
  // TransferSegmentMsg is meant to be packed into an 8 byte for CAN message.
  // The bit numbers on the right describe how it is packed into the 8 bytes
  struct TransferSegmentMsg {
    bool msg_type;                      // Bit 0: message type (1 bit)   
    uint16_t line_num;                  // Bits 1-15: line number (15 bits)
    uint8_t segment_num;                // Bits 16-19: segment number (4 bits)
    uint8_t total_segments;             // Bits 20-23: total number of segments (4 bits)
    char hex_data[MAX_HEX_CHUNK_SIZE];  // Bits 24-63: hex data (40 bits)
  };


  // ParsedHexLine is used to store the parsed hex line data after being unpacked and validated.
  struct ParsedHexLine {
    unsigned int byte_count; // Number of bytes in the data portion of the line
    unsigned int address;   // Address of the data
    unsigned int record_type; // Record type (0 for data, 1 for EOF, etc.)
    unsigned int data[16];   // Data bytes
    unsigned int checksum; // Checksum byte
    bool valid; // Flag to indicate if the line is valid
  };

  // AckMsg is used to acknowledge the receipt of a message.
  // It is meant to be packed into 8 bytes for CAN transfer back to the sender.
  // The bit numbers on the right describe how it is packed into the 8 bytes
  struct AckMsg
  {
    ResponseCode ack_msg_type;  // Bits 0-7: ResponseCode Code (1 byte)
    uint8_t data[6];            // Bits 8-47: data (6 bytes)
    // The last byte (bits 48-63) is used for checksum calculated at message send time
  };
  

  // --------------------------------------------------------------------------
  // Can Bus Message Handlers
  // --------------------------------------------------------------------------
  void handle_can_msg(uint8_t (&buf)[8]);
  
  TransferSegmentMsg unpack_transfer_segment_msg(uint8_t (&buf)[8]);
  bool process_transfer_segment_msg(TransferSegmentMsg &msg);
  
  TransferInitMsg unpack_transfer_init_msg(uint8_t (&buf)[8]);
  bool process_transfer_init_msg(TransferInitMsg &msg);
  
  
 
  // --------------------------------------------------------------------------
  // Hex Line Processing Functions
  // --------------------------------------------------------------------------
  // Main Hex line processing functions
  ResponseCode handle_received_hex_line();
  ParsedHexLine parse_and_validate_hex_line(const char (&buf)[MAX_HEX_LINE_SIZE]);
  bool process_hex_line(ParsedHexLine &hex_line);
  // Hex Record Processing Helper Functions
  bool process_hex_data_record(ParsedHexLine &hex_line);
  bool process_hex_eof_record(ParsedHexLine &hex_line);
  bool process_hex_extended_segment_address_record(ParsedHexLine &hex_line);
  bool process_hex_start_segment_address_record(ParsedHexLine &hex_line);
  bool process_hex_extended_linear_address_record(ParsedHexLine &hex_line);
  bool process_hex_start_linear_address_record(ParsedHexLine &hex_line);

  // --------------------------------------------------------------------------
  // Response Functions
  // --------------------------------------------------------------------------
  bool send_response(ResponseCode res, ErrorCode err = ErrorCode::NONE);
  bool pack_response(ResponseCode res, uint8_t (&buf)[8]);
  
  
  // --------------------------------------------------------------------------
  // Helper Functions
  // --------------------------------------------------------------------------
  bool are_all_segments_received();
  void add_hex_line_to_checksum();
  bool is_file_checksum_valid();
  void reset_cur_hex_line_buff();
  void clear_transfer_state();
  bool is_transfer_in_progress();
  bool is_file_transfer_complete();
  bool has_segment_timed_out();
  bool has_transfer_timed_out();
  void print_transfer_segment_msg(TransferSegmentMsg &msg);
  void print_transfer_init_msg(TransferInitMsg &msg);
  
  // ----------------------------------------------------------------------------
  // Main Functions
  // ----------------------------------------------------------------------------
  void update();
  void abort_transfer();
  void init();



}

#endif