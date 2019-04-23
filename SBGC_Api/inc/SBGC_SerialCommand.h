#ifndef _SBGC_SERIALCOMMAND_H_
#define _SBGC_SERIALCOMMAND_H_

#include "SBGC_Commands.h"

typedef struct {
	uint8_t pos;
	uint8_t id;
	uint8_t data[SBGC_CMD_DATA_SIZE];
	uint8_t len;
} SBGC_SerialCommand_t;

void SBGC_SC_init(SBGC_SerialCommand_t* sc, uint8_t _id);
void SBGC_SC_reset(SBGC_SerialCommand_t *sc);
uint8_t SBGC_SC_checkLimit(SBGC_SerialCommand_t *sc);
uint16_t SBGC_SC_getBytesAvailable(SBGC_SerialCommand_t *sc);
uint8_t SBGC_SC_readByte(SBGC_SerialCommand_t *sc);
void SBGC_SC_writeByte(SBGC_SerialCommand_t *sc, uint8_t b);
int16_t SBGC_SC_readWord(SBGC_SerialCommand_t *sc);
void SBGC_SC_writeWord(SBGC_SerialCommand_t *sc, int16_t w);
int32_t SBGC_SC_readLong(SBGC_SerialCommand_t *sc);
void SBGC_SC_writeLong(SBGC_SerialCommand_t *sc, int32_t dw);
float SBGC_SC_readFloat(SBGC_SerialCommand_t *sc);
void SBGC_SC_writeFloat(SBGC_SerialCommand_t *sc, float f);
void SBGC_SC_readWordArr(SBGC_SerialCommand_t *sc, int16_t *arr, uint8_t size);
void SBGC_SC_writeWordArr(SBGC_SerialCommand_t *sc, int16_t *arr, uint8_t size);
void SBGC_SC_readBuff(SBGC_SerialCommand_t *sc, void *buf, uint8_t size);
void SBGC_SC_writeBuff(SBGC_SerialCommand_t *sc, const void *buf, uint8_t size);
void SBGC_SC_writeString(SBGC_SerialCommand_t *sc, const char *str);
void SBGC_SC_writeEmptyBuf(SBGC_SerialCommand_t *sc, uint8_t size);
void SBGC_SC_skipBytes(SBGC_SerialCommand_t *sc, uint8_t size);


#endif /* _SBGC_SERIALCOMMAND_H_ */
