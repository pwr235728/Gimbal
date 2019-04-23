#ifndef _SBGC_COMOBJ_H_
#define _SBGC_COMOBJ_H_

#include "SBGC.h"
// Communication interface

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

#endif /* _SBGC_COMOBJ_H_ */
