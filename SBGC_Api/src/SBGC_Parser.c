#include "SBGC_Parser.h"
#include "SBGC.h"

#include <inttypes.h>
#include <string.h>


void SBGC_Parser_init(SBGC_Parser_t *parser, SBGC_ComObj_t *comObj){
	parser->com_obj = comObj;
	parser->state = STATE_WAIT;
	parser->parser_error_count = 0;
}
void SBGC_Parser_reset(SBGC_Parser_t *parser){
	parser->state = STATE_WAIT;
}
void SBGC_Parser_onParseError(SBGC_Parser_t *parser, uint8_t error){
	parser->parser_error_count++;
}

void SBGC_Parser_initChecksum(SBGC_Parser_t *parser){
	parser->checksum = 0;
}
void SBGC_Parser_updateChecksum(SBGC_Parser_t *parser, uint8_t byte){
	parser->checksum += byte;
}

SBGC_Parser_state SBGC_Parser_proccesChar(SBGC_Parser_t *parser, uint8_t c){
	switch (parser->state) {

		case STATE_WAIT:
			if (c == SBGC_CMD_START_BYTE) {
				parser->state = STATE_GOT_MARKER;
			} else {
				SBGC_Parser_onParseError(parser, PARSER_ERROR_PROTOCOL);
			}
			break;

		case STATE_GOT_MARKER:
			SBGC_SC_init(&(parser->rx_cmd), c);
			parser->state = STATE_GOT_ID;
			break;

		case STATE_GOT_ID:
			parser->len = c;
			parser->state = STATE_GOT_LEN;
			break;

		case STATE_GOT_LEN:
			if (c == (uint8_t) parser->rx_cmd.id + parser->len
					&& parser->len <= sizeof(parser->rx_cmd.data)) {
				SBGC_Parser_initChecksum(parser);
				parser->state = ((parser->len == 0) ? STATE_GOT_DATA : STATE_GOT_HEADER);
			} else {
				SBGC_Parser_onParseError(parser, PARSER_ERROR_PROTOCOL);
				parser->state = STATE_WAIT;
			}
			break;

		case STATE_GOT_HEADER:
			parser->rx_cmd.data[parser->rx_cmd.len++] = c;
			SBGC_Parser_updateChecksum(parser, c);
			if (parser->rx_cmd.len == parser->len) {
				parser->state = STATE_GOT_DATA;
			}
			break;

		case STATE_GOT_DATA:
			parser->state = STATE_WAIT;
			if (c == parser->checksum) {
				return PARSER_STATE_COMPLETE;
			} else {
				SBGC_Parser_onParseError(parser, PARSER_ERROR_PROTOCOL);
			}
			break;
	}
	return PARSER_STATE_INCOMPLETE;
}
uint8_t SBGC_Parser_send(SBGC_Parser_t *parser, uint8_t cmd_id, void *data, uint16_t size){

	if(size > (SBGC_CMD_MAX_BYTES - SBGC_CMD_NON_PAYLOAD_BYTES)){
		return PARSER_ERROR_WRONG_CMD_SIZE;
	}

	if(parser->com_obj->getOutEmptySpace() < size + SBGC_CMD_NON_PAYLOAD_BYTES){
		return PARSER_ERROR_BUFFER_IS_FULL;
	}

	parser->com_obj->writeByte(SBGC_CMD_START_BYTE);
	parser->com_obj->writeByte(cmd_id);
	parser->com_obj->writeByte(size);
	parser->com_obj->writeByte(cmd_id+size);


	uint8_t payload_checksum = 0;;
	for(uint8_t i = 0; i< size; i++){
		uint8_t byte = ((uint8_t*)data)[i];
		parser->com_obj->writeByte(byte);
		payload_checksum += byte;
	}
	parser->com_obj->writeByte(payload_checksum);

	return 0; // PARSER_NO_ERROR
}
uint8_t SBGC_Parser_sendCommand(SBGC_Parser_t *parser, SBGC_SerialCommand_t sc){
	return SBGC_Parser_send(parser, sc.id, sc.data, sc.len);
}
SBGC_Parser_state SBGC_Parser_receiveCommand(SBGC_Parser_t *parser, SBGC_SerialCommand_t * sc_out)
{
	while(parser->com_obj->getBytesAvailable())
	{

		uint8_t rx_byte = parser->com_obj->readByte();
		if(SBGC_Parser_proccesChar(parser, rx_byte) == PARSER_STATE_COMPLETE){

			*sc_out = parser->rx_cmd;

			return PARSER_STATE_COMPLETE;
		}
	}

	return PARSER_STATE_INCOMPLETE;
}
uint16_t SBGC_Parser_getParseErrorCount(SBGC_Parser_t *parser){
	return parser->parser_error_count;
}
uint16_t SBGC_Parser_getOutEmptySpace(SBGC_Parser_t *parser){
	return (parser->com_obj)->getOutEmptySpace();
}
