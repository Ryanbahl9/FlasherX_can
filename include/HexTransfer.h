#ifndef HEXTRANSFER_H
#define HEXTRANSFER_H

#include "Arduino.h"
#define MAX_HEX_LINE_SIZE 45 // Max size of hex line data, in bytes
#define MAX_HEX_CHUNK_SIZE 5 // Max size of hex data in a chunk, in bytes
#define PAD 0xFF 


// TransferChunkMsg is meant to be packed into 8 bytes for CAN transfer.
// The bit numbers on the right describe how it is packed into the 8 bytes
// TransferChunkMsg hold a single chunk of a hex line
struct TransferChunkMsg {
  bool msg_type;                      // Bit 0: message type (1 bit)   
  uint16_t line_num;                  // Bits 1–15: line number (15 bits)
  uint8_t chunk_num;                  // Bits 16–19: chunk number (4 bits)
  uint8_t total_chunks;               // Bits 20–23: total number of chunks (4 bits)
  char hex_data[MAX_HEX_CHUNK_SIZE];  // Bits 24–63: hex data (40 bits)
};

// TransferInitMsg is meant to be packed into 8 bytes for CAN transfer.
// The bit numbers on the right describe how it is packed into the 8 bytes
// TransferInitMsg is the first message sent to initialize the transfer
struct TransferInitMsg {
  // Total of 64 bits (8 bytes):
  uint64_t msg_type           : 1;   // Bit 0: message type (1 bit)
  uint64_t line_count         : 15;  // Bits 1–15: total number of lines in the hex file (15 bits)
  uint64_t file_checksum      : 32;  // Bits 16–47: total file checksum (32 bits)
  uint64_t init_msg_checksum  : 16;  // Bits 48–63: checksum of the init message (16 bits)
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
  bool eof;        // Flag to indicate if EOF has been reached
  size_t lines;    // Number of hex records received
};


class HexTransfer
{
private:
  size_t  cur_hex_line_num;         // Current line number
  size_t  cur_hex_line_chunk_count; // Number of chunks in the current line
  bool    chunks_received[9];       // Array to track received chunks
  bool    all_chunks_received;      // Flag to indicate if all chunks have been received
  char cur_hex_line_data[MAX_HEX_LINE_SIZE]; // Hex data, max MAX_HEX_LINE_SIZE bytes
  
  void send_hex_line_received_acknowledgment(size_t line_num, bool has_error);
  void send_hex_transfer_init_acknowledgment(bool has_error);
  void send_hex_transfer_complete_acknowledgment(bool has_error);
  
  TransferChunkMsg HexTransfer::unpack_transfer_chunk_msg(uint8_t (&buf)[8]);
  
  bool process_transfer_chunk_msg(TransferChunkMsg &msg);
  ParsedHexLine parse_and_validate_hex_line(const char (&buf)[MAX_HEX_LINE_SIZE]);
  void process_hex_line(ParsedHexLine &hex_line);
  
public:
  HexTransfer(/* args */);
  void process_can_msg(uint8_t (&buf)[8]);
  
  ~HexTransfer();
};



#endif