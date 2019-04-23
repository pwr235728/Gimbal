
#include "../sbgc-api/include/SBGC_parser.h"

#include <inttypes.h>
#include <string.h>
#include "../include/SBGC_command.h"

#define SBGC_CMD_START_BYTE '>'

void SC_init(SerialCommand_t* sc, uint8_t _id){
	sc->id =_id;
	sc->len = 0;
	sc->pos = 0;
}
void SC_reset(SerialCommand_t *sc){
	sc->len = 0;
	sc->pos = 0;
}

uint8_t SC_checkLimit(SerialCommand_t *sc){
	return sc->len == sc->pos;
}
uint16_t SC_getBytesAvailable(SerialCommand_t *sc){
	return sc->len - sc->pos;
}
uint8_t SC_readByte(SerialCommand_t *sc){
	if(sc->pos < sc->len){
		return sc->data[sc->pos++];
	}else{
		sc->pos++;
		return 0;
	}
}
void SC_writeByte(SerialCommand_t *sc, uint8_t b){
	if(sc->len < sizeof(sc->data)){
		sc->data[sc->len++] = b;
	}
}
int16_t SC_readWord(SerialCommand_t *sc){
	return (uint16_t)SC_readByte(sc) + ((uint16_t)SC_readByte(sc)<<8);
}
void SC_writeWord(SerialCommand_t *sc, int16_t w){
	SC_writeByte(sc, w);		// low byte
	SC_writeByte(sc, w>>8);	// high byte
}
int32_t SC_readLong(SerialCommand_t *sc){
	return (int32_t)SC_readByte(sc) + ((int32_t)SC_readByte(sc)<<8) +
			((int32_t)SC_readByte(sc)<<16) + ((int32_t)SC_readByte(sc)<<24);
}
void SC_writeLong(SerialCommand_t *sc, int32_t dw){
	SC_writeWord(sc, dw);			// low word
	SC_writeWord(sc, dw >> 16);	// high word
}
float SC_readFloat(SerialCommand_t *sc){
	float f;
	SC_readBuff(sc, &f, sizeof(float));
	return f;
}
void SC_writeFloat(SerialCommand_t *sc, float f){
	SC_writeBuff(sc, &f, sizeof(float));
}
void SC_readWordArr(SerialCommand_t *sc, int16_t *arr, uint8_t size){
	#ifdef SYS_LITTLE_ENDIAN
	SC_readBuff(sc, arr, (size*2));
	#else
	for(uint8_t i=0; i<size; i++){
		arr[i] = SC_readWord(sc);
	}
	#endif
}
void SC_writeWordArr(SerialCommand_t *sc, int16_t *arr, uint8_t size){
	#ifdef SYS_LITTLE_ENDIAN
	SC_writeBuff(sc, arr, (size*2));
	#else
	for(uint8_t i=0; i<size; i++){
		SC_writeWord(sc, arr[i]);
		}
	#endif
}
void SC_readBuff(SerialCommand_t *sc, void *buf, uint8_t size){
	for(uint8_t i=0; i<size; i++){
		((uint8_t*)buf)[i] = SC_readByte(sc);
	}
}
void SC_writeBuff(SerialCommand_t *sc, const void *buf, uint8_t size){
	for(uint8_t i=0; i<size; i++){
		SC_writeByte(sc, ((uint8_t*)buf)[i]);
	}
}
void SC_writeString(SerialCommand_t *sc, const char *str){
	uint8_t len = strlen(str);
	SC_writeByte(sc, len);
	SC_writeBuff(sc, str, len);
}
void SC_writeEmptyBuf(SerialCommand_t *sc, uint8_t size){
	while(size-- > 0){
		SC_writeByte(sc, 0);
	}
}
void SC_skipBytes(SerialCommand_t *sc, uint8_t size){
	while(size-- > 0){
		SC_readByte(sc);
	}
}




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
			SC_init(&(parser->rx_cmd), c);
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
				parser->state = (
						(parser->len == 0) ? STATE_GOT_DATA : STATE_GOT_HEADER);
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
uint8_t SBGC_Parser_sendCommand(SBGC_Parser_t *parser, uint8_t cmd_id, void *data, uint16_t size){

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
uint8_t SBGC_Parser_sendCmd(SBGC_Parser_t *parser, SerialCommand_t sc){
	return SBGC_Parser_sendCommand(parser, sc.id, sc.data, sc.len);
}
uint16_t SBGC_Parser_getParseErrorCount(SBGC_Parser_t *parser){
	return parser->parser_error_count;
}
uint16_t SBGC_Parser_getOutEmptySpace(SBGC_Parser_t *parser){
	return (parser->com_obj)->getOutEmptySpace();
}
