#include "SBGC_SerialCommand.h"
#include "SBGC.h"

#include <string.h>

void SBGC_SC_init(SBGC_SerialCommand_t* sc, uint8_t _id){
	sc->id =_id;
	sc->len = 0;
	sc->pos = 0;
}
void SBGC_SC_reset(SBGC_SerialCommand_t *sc){
	sc->len = 0;
	sc->pos = 0;
}

uint8_t SBGC_SC_checkLimit(SBGC_SerialCommand_t *sc){
	return sc->len == sc->pos;
}
uint16_t SBGC_SC_getBytesAvailable(SBGC_SerialCommand_t *sc){
	return sc->len - sc->pos;
}
uint8_t SBGC_SC_readByte(SBGC_SerialCommand_t *sc){
	if(sc->pos < sc->len){
		return sc->data[sc->pos++];
	}else{
		sc->pos++;
		return 0;
	}
}
void SBGC_SC_writeByte(SBGC_SerialCommand_t *sc, uint8_t b){
	if(sc->len < sizeof(sc->data)){
		sc->data[sc->len++] = b;
	}
}
int16_t SBGC_SC_readWord(SBGC_SerialCommand_t *sc){
	return (uint16_t)SBGC_SC_readByte(sc) + ((uint16_t)SBGC_SC_readByte(sc)<<8);
}
void SBGC_SC_writeWord(SBGC_SerialCommand_t *sc, int16_t w){
	SBGC_SC_writeByte(sc, w);		// low byte
	SBGC_SC_writeByte(sc, w>>8);	// high byte
}
int32_t SBGC_SC_readLong(SBGC_SerialCommand_t *sc){
	return (int32_t)SBGC_SC_readByte(sc) + ((int32_t)SBGC_SC_readByte(sc)<<8) +
			((int32_t)SBGC_SC_readByte(sc)<<16) + ((int32_t)SBGC_SC_readByte(sc)<<24);
}
void SBGC_SC_writeLong(SBGC_SerialCommand_t *sc, int32_t dw){
	SBGC_SC_writeWord(sc, dw);			// low word
	SBGC_SC_writeWord(sc, dw >> 16);	// high word
}
float SBGC_SC_readFloat(SBGC_SerialCommand_t *sc){
	float f;
	SBGC_SC_readBuff(sc, &f, sizeof(float));
	return f;
}
void SBGC_SC_writeFloat(SBGC_SerialCommand_t *sc, float f){
	SBGC_SC_writeBuff(sc, &f, sizeof(float));
}
void SBGC_SC_readWordArr(SBGC_SerialCommand_t *sc, int16_t *arr, uint8_t size){
	#ifdef SYS_LITTLE_ENDIAN
	SBGC_SC_readBuff(sc, arr, (size*2));
	#else
	for(uint8_t i=0; i<size; i++){
		arr[i] = SBGC_SC_readWord(sc);
	}
	#endif
}
void SBGC_SC_writeWordArr(SBGC_SerialCommand_t *sc, int16_t *arr, uint8_t size){
	#ifdef SYS_LITTLE_ENDIAN
	SBGC_SC_writeBuff(sc, arr, (size*2));
	#else
	for(uint8_t i=0; i<size; i++){
		SBGC_SC_writeWord(sc, arr[i]);
		}
	#endif
}
void SBGC_SC_readBuff(SBGC_SerialCommand_t *sc, void *buf, uint8_t size){
	for(uint8_t i=0; i<size; i++){
		((uint8_t*)buf)[i] = SBGC_SC_readByte(sc);
	}
}
void SBGC_SC_writeBuff(SBGC_SerialCommand_t *sc, const void *buf, uint8_t size){
	for(uint8_t i=0; i<size; i++){
		SBGC_SC_writeByte(sc, ((uint8_t*)buf)[i]);
	}
}
void SBGC_SC_writeString(SBGC_SerialCommand_t *sc, const char *str){
	uint8_t len = strlen(str);
	SBGC_SC_writeByte(sc, len);
	SBGC_SC_writeBuff(sc, str, len);
}
void SBGC_SC_writeEmptyBuf(SBGC_SerialCommand_t *sc, uint8_t size){
	while(size-- > 0){
		SBGC_SC_writeByte(sc, 0);
	}
}
void SBGC_SC_skipBytes(SBGC_SerialCommand_t *sc, uint8_t size){
	while(size-- > 0){
		SBGC_SC_readByte(sc);
	}
}
