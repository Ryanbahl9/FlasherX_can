#ifndef HEXTRANSFER_H
#define HEXTRANSFER_H

#include "Arduino.h"
#include <FastCRC.h>
extern "C" {
  #include "FlashTxx.h"		// TLC/T3x/T4x/TMM flash primitives
}

namespace HexTransfer {
  

  #define MAX_HEX_LINE_SIZE 45      // Max size of hex line data, in bytes
  #define MAX_HEX_CHUNK_SIZE 5      // Max size of hex data in a segment, in bytes
  #define MAX_CHUNKS_PER_HEX_LINE 9 // 45/5 = 9
  #define PAD 0xFF 
  
  #define HEX_LINE_TIMEOUT_LEN 5000     // Timeout for receiving hex line segments, in ms
  #define INACTIVITY_TIMEOUT_LEN 15000  // Timeout for inactivity, in ms

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
    uint16_t line_num;                  // Bits 1–15: line number (15 bits)
    uint8_t segment_num;                // Bits 16–19: segment number (4 bits)
    uint8_t total_segments;             // Bits 20–23: total number of segments (4 bits)
    char hex_data[MAX_HEX_CHUNK_SIZE];  // Bits 24–63: hex data (40 bits)
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

  // HexFileInfo is used to store information about the hex file being transferred.
  struct HexFileInfo {
    uint32_t base;   // Base address to be added to the hex line address
    uint32_t min;    // Minimum address in the hex file
    uint32_t max;    // Maximum address in the hex file
    bool eof_received;        // Flag to indicate if EOF has been reached
    size_t lines;    // Number of hex records received
  };

  /*
   * Acks
   * - TRANSFER_INIT_OK
   * - HEX_LINE_OK
   * - HEX_LINE_SEGMENT_TIMEOUT
   * - HEX_FILE_SUCCESS
   * Errors
   * - TRANSFER_INIT_CHECKSUM_ERROR
   * - HEX_LINE_PARSE_ERROR
   * - HEX_LINE_PROCESSING_ERROR
   * - HEX_LINE_SEGMENT_TIMEOUT
   * - INACTIVITY_TIMEOUT
   * - HEX_FILE_CHECKSUM_ERROR
  */

  // -----------------------------------------------------------------
  // Hex Transfer Result Enum
  // -----------------------------------------------------------------
  enum HexResult {
    /**
     * NONE
     *   Cause:             No action taken.
     *   Result:            No Acknowledgement or Error message sent.
     *   Expected Response: None.
    */
    NONE = 0,
    
    /**
     * TRANSFER_INIT_OK
     *   Cause:             Init message and its checksum were valid.
     *   Result:            Abort all current transfers, set transfer in progress
     *                      to true, and get reset global vars.
     *   Expected Response: Start sending first hex line.
    */
    TRANSFER_INIT_OK,

    /**
     * TRANSFER_INIT_CHECKSUM_ERROR
     *   Cause:             The checksum for the Transfer Init Msg has failed. 
     *   Result:            Nothing, discard message.
     *   Expected Response: Resend message.
    */  
    TRANSFER_INIT_CHECKSUM_ERROR,
      
    /**
     * HEX_LINE_OK
     *   Cause:             Hex Line was received, parsed, and processed.
     *   Result:            Hex Line vars are clears and readied for the next line.
     *   Expected Response: Send next hex line.
    */  
    HEX_LINE_OK,
    
    /**
     * HEX_LINE_PARSE_ERROR
     *   Cause:             Couldn't parse received hex line, or the line failed
     *                      failed its checksum.
     *   Result:            Discard all segments and clear Hex Line vars.
     *   Expected Response: Resend all segments for current Hex Line.
    */  
    HEX_LINE_PARSE_ERROR,
    
    /**
     * HEX_LINE_PROCESSING_ERROR
     *   Cause:             Couldn't process hex line. 
     *   Result:            Discard all segments and clear Hex Line vars.
     *   Expected Response: Resend all segments for current Hex Line.
    */  
    HEX_LINE_PROCESSING_ERROR,
    
    /**
     * HEX_LINE_SEGMENT_TIMEOUT
     *   Cause:             The hex segment timeout was hit. 
     *   Result:            Send error and wait to receive missing segment.
     *   Expected Response: Resend all segments for current Hex Line.
     *                      More segments than necessary will be sent, but whatever.
    */  
    HEX_LINE_SEGMENT_TIMEOUT,
    
    /**
     * INACTIVITY_TIMEOUT
     *   Cause:             No can messages for the hex transfer have been 
     *                      received for a set number of seconds. 
     *   Result:            Abort Transfer.
     *   Expected Response: Start new transfer with new transfer init message,
     *                      or give up.
    */  
    INACTIVITY_TIMEOUT,
    
    /**
     * HEX_FILE_SUCCESS
     *   Cause:             All lines have been received and checksum passed. 
     *   Result:            Set Hex File Received Flag. (Higher Level Class will
     *                      auto-start the flash)
     *   Expected Response: Nothing
    */
    HEX_FILE_SUCCESS,

    /**
     * HEX_FILE_CHECKSUM_ERROR
     *   Cause:             All lines have been received but whole file checksum
     *                      doesn't the one received in the init message. 
     *   Result:            Abort Transfer.
     *   Expected Response: Start new transfer with new transfer init message,
     *                      or give up.
    */  
    HEX_FILE_CHECKSUM_ERROR
  };


  // --------------------------------------------------------------------------
  // Can Bus Message Handlers
  // --------------------------------------------------------------------------
  void process_can_msg(uint8_t (&buf)[8]);
  
  TransferSegmentMsg unpack_transfer_segment_msg(uint8_t (&buf)[8]);
  bool process_transfer_segment_msg(TransferSegmentMsg &msg);
  
  TransferInitMsg unpack_transfer_init_msg(uint8_t (&buf)[8]);
  bool process_transfer_init_msg(TransferInitMsg &msg);

  bool send_response(HexResult res);
 
  // --------------------------------------------------------------------------
  // Hex Line Processing Functions
  // --------------------------------------------------------------------------
  // Main Hex line processing functions
  HexResult handle_received_hex_line();
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
  // Helper Functions
  // --------------------------------------------------------------------------
  bool are_all_segments_received();
  void add_hex_line_to_checksum();
  bool is_file_checksum_valid();
  void reset_cur_hex_line_buff();
  void reset_hex_file_info();  
  bool is_transfer_in_progress();
  bool is_file_transfer_complete();
  bool has_segment_timed_out();
  bool has_transfer_timed_out();
  
  // ----------------------------------------------------------------------------
  // Main Functions
  // ----------------------------------------------------------------------------
  void update();
  void abort_transfer();
  void init();



}

#endif