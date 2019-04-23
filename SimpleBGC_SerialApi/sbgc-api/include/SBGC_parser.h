/* 
	SimpleBGC Serial API  library - input data parser
	More info: http://www.basecamelectronics.com/serialapi/

  Copyright (c) 2014-2015 Aleksei Moskalenko
  All rights reserved.
	
	See license info in the SBGC.h
*/   

#ifndef  __SBGC_parser__
#define  __SBGC_parser__

#include <inttypes.h>
#include <string.h>
#include "SBGC_command.h"


#define SBGC_CMD_START_BYTE '>'

typedef enum {
	PARSER_NO_ERROR=0,
	PARSER_ERROR_PROTOCOL=1,
	PARSER_ERROR_WRONG_CMD_SIZE=2,
	PARSER_ERROR_BUFFER_IS_FULL=3,
	PARSER_ERROR_WRONG_DATA_SIZE=4,
} SBGC_parser_errors;

// class SBGC_IOStream and SerialCommand: SBGC_IOStream
// SC stand for SerialCommand

typedef struct {
	uint8_t pos;
	uint8_t id;
	uint8_t data[SBGC_CMD_DATA_SIZE];
	uint8_t len;
} SerialCommand_t;

void SC_init(SerialCommand_t* sc, uint8_t _id);
void SC_reset(SerialCommand_t *sc);
uint8_t SC_checkLimit(SerialCommand_t *sc);
uint16_t SC_getBytesAvailable(SerialCommand_t *sc);
uint8_t SC_readByte(SerialCommand_t *sc);
void SC_writeByte(SerialCommand_t *sc, uint8_t b);
int16_t SC_readWord(SerialCommand_t *sc);
void SC_writeWord(SerialCommand_t *sc, int16_t w);
int32_t SC_readLong(SerialCommand_t *sc);
void SC_writeLong(SerialCommand_t *sc, int32_t dw);
float SC_readFloat(SerialCommand_t *sc);
void SC_writeFloat(SerialCommand_t *sc, float f);
void SC_readWordArr(SerialCommand_t *sc, int16_t *arr, uint8_t size);
void SC_writeWordArr(SerialCommand_t *sc, int16_t *arr, uint8_t size);
void SC_readBuff(SerialCommand_t *sc, void *buf, uint8_t size);
void SC_writeBuff(SerialCommand_t *sc, const void *buf, uint8_t size);
void SC_writeString(SerialCommand_t *sc, const char *str);
void SC_writeEmptyBuf(SerialCommand_t *sc, uint8_t size);
void SC_skipBytes(SerialCommand_t *sc, uint8_t size);



// class SBGC_ComObj

typedef struct{
	// Return the number of bytes received in input buffer
	uint16_t (*getBytesAvailable)(void);

	// Read byte from the input stream
	uint8_t (*readByte)(void);

	// Write byte to the output stream
	void (*writeByte)(uint8_t);

	// Return the space available in the output buffer. Return 0xFFFF if unknown.
	uint16_t (*getOutEmptySpace)(void);
}SBGC_ComObj_t;


// class SBGC_Parser

typedef struct{
	enum{
		STATE_WAIT,
		STATE_GOT_MARKER,
		STATE_GOT_ID,
		STATE_GOT_LEN,
		STATE_GOT_HEADER,
		STATE_GOT_DATA
	} state;

	SBGC_ComObj_t *com_obj;

	uint16_t len;
	uint8_t checksum;
	uint16_t parser_error_count;


	// Received command
	SerialCommand_t rx_cmd;
}SBGC_Parser_t;

typedef enum{
	PARSER_STATE_INCOMPLETE = 0,
	PARSER_STATE_COMPLETE
}SBGC_Parser_state;

void SBGC_Parser_init(SBGC_Parser_t *parser, SBGC_ComObj_t *comObj);
void SBGC_Parser_reset(SBGC_Parser_t *parser);
void SBGC_Parser_onParseError(SBGC_Parser_t *parser, uint8_t error);

void SBGC_Parser_initChecksum(SBGC_Parser_t *parser);
void SBGC_Parser_updateChecksum(SBGC_Parser_t *parser, uint8_t byte);

SBGC_Parser_state SBGC_Parser_proccesChar(SBGC_Parser_t *parser, uint8_t c);
uint8_t SBGC_Parser_sendCommand(SBGC_Parser_t *parser, uint8_t cmd_id, void *data, uint16_t size);
uint8_t SBGC_Parser_sendCmd(SBGC_Parser_t *parser, SerialCommand_t sc);
uint16_t SBGC_Parser_getParseErrorCount(SBGC_Parser_t *parser);
uint16_t SBGC_Parser_getOutEmptySpace(SBGC_Parser_t *parser);

#endif //__SBGC_parser__
