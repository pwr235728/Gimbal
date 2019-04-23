/* 
	SimpleBGC Serial API  library - helpers to pack and parse command data
	More info: http://www.basecamelectronics.com/serialapi/

	Copyright (c) 2014-2015 Aleksei Moskalenko
	All rights reserved.
	
	See license info in the SBGC.h
*/   


#include "../include/SBGC_cmd_helpers.h"
#include <string.h>

#include <SBGC.h>

/* Packs command structure to SerialCommand object */
void SBGC_cmd_control_pack(SBGC_cmd_control_t *p, SerialCommand_t *cmd) {
	SC_init(cmd, SBGC_CMD_CONTROL);

	SC_writeByte(cmd, p->mode); 	// ROLL axis
	SC_writeByte(cmd, p->mode); 	// PITCH axis
	SC_writeByte(cmd, p->mode);	// YAW axis
	SC_writeWord(cmd, p->speedROLL);
	SC_writeWord(cmd, p->angleROLL);
	SC_writeWord(cmd, p->speedPITCH);
	SC_writeWord(cmd, p->anglePITCH);
	SC_writeWord(cmd, p->speedYAW);
	SC_writeWord(cmd, p->angleYAW);
}

uint8_t SBGC_cmd_control_send(SBGC_cmd_control_t *p, SBGC_Parser_t *parser) {
	SerialCommand_t cmd;
	SBGC_cmd_control_pack(p, &cmd);
	return SBGC_Parser_sendCmd(parser, cmd);
}

/* Packs command structure to SerialCommand object */
void SBGC_cmd_control_ext_pack(SBGC_cmd_control_ext_t *p, SerialCommand_t *cmd) {
	SC_init(cmd, SBGC_CMD_CONTROL);

	SC_writeBuf(cmd, p->mode, 3);
	for (uint8_t i = 0; i < 3; i++) {
		SC_writeWord(cmd, p->data[i].speed);
		SC_writeWord(cmd, p->data[i].angle);
	}

}


/* Packs command structure to SerialCommand object */
void SBGC_cmd_servo_out_pack(SBGC_cmd_servo_out_t *p, SerialCommand_t *cmd) {
	SC_init(cmd, SBGC_CMD_SERVO_OUT);

	for (uint8_t i = 0; i < 8; i++) {
		SC_writeWord(cmd, p->servo[i]);
	}
}


/*
* Unpacks SerialCommand object to command structure.
* Returns 0 on success, PARSER_ERROR_XX code on fail.
*/
uint8_t SBGC_cmd_realtime_data_unpack(SBGC_cmd_realtime_data_t *p, SerialCommand_t *cmd) {

	for (uint8_t i = 0; i < 3; i++) {
		p->sensor_data[i].acc_data = SC_readWord(cmd);
		p->sensor_data[i].gyro_data = SC_readWord(cmd);
	}
	p->serial_error_cnt = SC_readWord(cmd);
	p->system_error = SC_readWord(cmd);
	SC_skipBytes(cmd, 4); // reserved
	SC_readWordArr(cmd, p->rc_raw_data, SBGC_RC_NUM_CHANNELS);
	SC_readWordArr(cmd, p->imu_angle, 3);
	SC_readWordArr(cmd, p->frame_imu_angle, 3);
	SC_readWordArr(cmd, p->target_angle, 3);
	p->cycle_time_us = SC_readWord(cmd);
	p->i2c_error_count = SC_readWord(cmd);
	SC_readByte(cmd); // reserved
	p->battery_voltage = SC_readWord(cmd);
	p->state_flags1 = SC_readByte(cmd);
	p->cur_imu = SC_readByte(cmd);
	p->cur_profile = SC_readByte(cmd);
	SC_readBuf(cmd, p->motor_power, 3);

	if (cmd->id == SBGC_CMD_REALTIME_DATA_4) {
		SC_readWordArr(cmd, p->rotor_angle, 3);
		SC_readByte(cmd); // reserved
		SC_readWordArr(cmd, p->balance_error, 3);
		p->current = SC_readWord(cmd);
		SC_readWordArr(cmd, p->magnetometer_data, 3);
		p->imu_temp_celcius = SC_readByte(cmd);
		p->frame_imu_temp_celcius = SC_readByte(cmd);
		SC_skipBytes(cmd, 38);
	}

	if (SC_checkLimit(cmd))
		return 0;
	else
		return PARSER_ERROR_WRONG_DATA_SIZE;
}
