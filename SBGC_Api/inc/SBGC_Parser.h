#ifndef _SBGC_PARSER_H_
#define _SBGC_PARSER_H_

#include "SBGC_ComObj.h"
#include "SBGC.h"

#define SBGC_CMD_START_BYTE '>'

typedef enum {
	PARSER_NO_ERROR=0,
	PARSER_ERROR_PROTOCOL=1,
	PARSER_ERROR_WRONG_CMD_SIZE=2,
	PARSER_ERROR_BUFFER_IS_FULL=3,
	PARSER_ERROR_WRONG_DATA_SIZE=4,
} SBGC_parser_errors;

typedef enum{
	PARSER_STATE_INCOMPLETE = 0,
	PARSER_STATE_COMPLETE
}SBGC_Parser_state;

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
	SBGC_SerialCommand_t rx_cmd;
}SBGC_Parser_t;

void SBGC_Parser_init(SBGC_Parser_t *parser, SBGC_ComObj_t *comObj);
void SBGC_Parser_reset(SBGC_Parser_t *parser);
void SBGC_Parser_onParseError(SBGC_Parser_t *parser, uint8_t error);

void SBGC_Parser_initChecksum(SBGC_Parser_t *parser);
void SBGC_Parser_updateChecksum(SBGC_Parser_t *parser, uint8_t byte);

SBGC_Parser_state SBGC_Parser_proccesChar(SBGC_Parser_t *parser, uint8_t c);
uint8_t SBGC_Parser_send(SBGC_Parser_t *parser, uint8_t cmd_id, void *data, uint16_t size);
uint8_t SBGC_Parser_sendCommand(SBGC_Parser_t *parser, SBGC_SerialCommand_t sc);
SBGC_Parser_state SBGC_Parser_receiveCommand(SBGC_Parser_t *parser, SBGC_SerialCommand_t * sc_out);
uint16_t SBGC_Parser_getParseErrorCount(SBGC_Parser_t *parser);
uint16_t SBGC_Parser_getOutEmptySpace(SBGC_Parser_t *parser);


#endif /* _SBGC_PARSER_H_ */
