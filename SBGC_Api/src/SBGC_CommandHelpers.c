#include "SBGC_CommandHelpers.h"
#include "SBGC.h"

// Control

void _SBGC_cmd_control_pack(SBGC_CMD_Control_t *ctrl, SBGC_SerialCommand_t *cmd)
{
	SBGC_SC_init(cmd, SBGC_CMD_CONTROL);
	SBGC_SC_writeByte(cmd, ctrl->mode);

	SBGC_SC_writeWord(cmd, ctrl->speedRoll);
	SBGC_SC_writeWord(cmd, ctrl->angleRoll);
	SBGC_SC_writeWord(cmd, ctrl->speedPitch);
	SBGC_SC_writeWord(cmd, ctrl->anglePitch);
	SBGC_SC_writeWord(cmd, ctrl->speedYaw);
	SBGC_SC_writeWord(cmd, ctrl->angleYaw);
}


void _SBGC_cmd_control_ext_pack(SBGC_CMD_ControlExt_t *ctrlExt, SBGC_SerialCommand_t *cmd)
{
	SBGC_SC_init(cmd, SBGC_CMD_CONTROL);
	SBGC_SC_writeByte(cmd, ctrlExt->mode[ROLL]);
	SBGC_SC_writeByte(cmd, ctrlExt->mode[PITCH]);
	SBGC_SC_writeByte(cmd, ctrlExt->mode[YAW]);

	SBGC_SC_writeWord(cmd, ctrlExt->data[ROLL].speed);
	SBGC_SC_writeWord(cmd, ctrlExt->data[ROLL].angle);
	SBGC_SC_writeWord(cmd, ctrlExt->data[PITCH].speed);
	SBGC_SC_writeWord(cmd, ctrlExt->data[PITCH].angle);
	SBGC_SC_writeWord(cmd, ctrlExt->data[YAW].speed);
	SBGC_SC_writeWord(cmd, ctrlExt->data[YAW].angle);
}

uint8_t SBGC_cmd_control_send(SBGC_CMD_Control_t *ctrl, SBGC_Parser_t *parser)
{
	SBGC_SerialCommand_t cmd;
	_SBGC_cmd_control_pack(ctrl, &cmd);
	return SBGC_Parser_sendCommand(parser, cmd);
}
uint8_t SBGC_cmd_control_ext_send(SBGC_CMD_ControlExt_t *ctrlExt, SBGC_Parser_t *parser)
{
	SBGC_SerialCommand_t cmd;
	_SBGC_cmd_control_ext_pack(ctrlExt, &cmd);
	return SBGC_Parser_sendCommand(parser, cmd);
}



// Realtime data

uint8_t SBGC_cmd_realtime_data_3_unpack(SBGC_RealtimeData_3_t *rtData3, SBGC_SerialCommand_t *cmd)
{
	for(uint8_t i=0; i< 3; i++){
		rtData3->sensor_data[i].acc_data = SBGC_SC_readWord(cmd);
		rtData3->sensor_data[i].gyro_data = SBGC_SC_readWord(cmd);
	}

	rtData3->serial_err_cnt = SBGC_SC_readWord(cmd);
	rtData3->system_error = SBGC_SC_readWord(cmd);
	rtData3->system_sub_error = SBGC_SC_readByte(cmd);
	SBGC_SC_skipBytes(cmd, 3);
	SBGC_SC_readWordArr(cmd, rtData3->rc_raw_data, SBGC_RC_NUM_CHANNELS);
	SBGC_SC_readWordArr(cmd, rtData3->imu_angle, 3);
	SBGC_SC_readWordArr(cmd, rtData3->frame_imu_angle, 3);
	SBGC_SC_readWordArr(cmd, rtData3->target_angles, 3);
	rtData3->cycle_time = SBGC_SC_readWord(cmd);
	rtData3->i2c_error_count = SBGC_SC_readWord(cmd);
	rtData3->error_code = SBGC_SC_readByte(cmd);
	rtData3->bat_level = SBGC_SC_readWord(cmd);
	rtData3->rt_data_flags = SBGC_SC_readByte(cmd);
	rtData3->cur_imu = SBGC_SC_readByte(cmd);
	rtData3->cur_profile = SBGC_SC_readByte(cmd);
	SBGC_SC_readBuff(cmd, rtData3->motor_power, 3);

	if(cmd->id != SBGC_CMD_REALTIME_DATA && cmd->id != SBGC_CMD_REALTIME_DATA_3){
		return PARSER_NO_ERROR; // 0
	}

	if(SBGC_SC_checkLimit(cmd)){
		return PARSER_NO_ERROR; // 0
	}else{
		return PARSER_ERROR_WRONG_DATA_SIZE;
	}
}

uint8_t SBGC_cmd_realtime_data_4_unpack(SBGC_RealtimeData_4_t *rtData4, SBGC_SerialCommand_t *cmd)
{
	SBGC_cmd_realtime_data_3_unpack(&(rtData4->cmd_realtime_data_3), cmd);

	if(cmd->id == SBGC_CMD_REALTIME_DATA_4){
		SBGC_SC_readWordArr(cmd, rtData4->stator_rotor_angle, 3);
		SBGC_SC_readByte(cmd); // reserved
		SBGC_SC_readWordArr(cmd, rtData4->balance_error, 3);
		rtData4->current = SBGC_SC_readWord(cmd);
		SBGC_SC_readWordArr(cmd, rtData4->mag_data, 3);
		rtData4->imu_temperature = SBGC_SC_readByte(cmd);
		rtData4->frame_imu_temperature = SBGC_SC_readByte(cmd);
		rtData4->imu_g_err = SBGC_SC_readByte(cmd);
		rtData4->imu_h_err = SBGC_SC_readByte(cmd);
		SBGC_SC_readWordArr(cmd, rtData4->motor_out, 3);
		SBGC_SC_skipBytes(cmd, 30);
	}

	if(SBGC_SC_checkLimit(cmd)){
		return PARSER_NO_ERROR; // 0
	}else{
		return PARSER_ERROR_WRONG_DATA_SIZE;
	}
}

