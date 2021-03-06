#ifndef  _SBGC_
#define  _SBGC_

// !!!!Define the endianess of your machine!!!! 
#define SYS_LITTLE_ENDIAN


#include <inttypes.h>
#include <stdint.h>

#include "SBGC_Commands.h"
#include "SBGC_SerialCommand.h"
#include "SBGC_ComObj.h"
#include "SBGC_Parser.h"
#include "SBGC_Rc.h"
#include "SBGC_CommandHelpers.h"

// Error codes in command response
#define SBGC_ERR_CMD_SIZE 			1
#define SBGC_ERR_WRONG_PARAMS 		2
#define SBGC_ERR_GET_DEVICE_ID  	3
#define SBGC_ERR_CRYPTO				4
#define SBGC_ERR_CALIBRATE_BAT 		5
#define SBGC_ERR_UNKNOWN_COMMAND 	6


// System error flags that controller may set
#define SBGC_SYS_ERR_NO_SENSOR 		(1<<0)
#define SBGC_SYS_ERR_CALIB_ACC 		(1<<1)
#define SBGC_SYS_ERR_SET_POWER 		(1<<2)
#define SBGC_SYS_ERR_CALIB_POLES 	(1<<3)
#define SBGC_SYS_ERR_PROTECTION 	(1<<4)
#define SBGC_SYS_ERR_SERIAL 		(1<<5)
#define SBGC_SYS_ERR_BAT_LOW 		(1<<6)
#define SBGC_SYS_ERR_BAT_CRITICAL 	(1<<7)
#define SBGC_SYS_ERR_GUI_VERSION 	(1<<8)
#define SBGC_SYS_ERR_MISS_STEPS 	(1<<9)
#define SBGC_SYS_ERR_SYSTEM 		(1<<10)
#define SBGC_SYS_ERR_EMERGENCY_STOP (1<<11)

// Specifies the reason of emergency stop
#define SBGC_SYS_SUB_ERR_I2C_ERROS 					1
#define SBGC_SYS_SUB_ERR_DRV_OTW 					2
#define SBGC_SYS_SUB_ERR_DRV_FAULT					3
#define SBGC_SYS_SUB_ERR_ENCODER_IMU_ANGLE  		4
#define SBGC_SYS_SUB_ERR_CALIBRATION_FAILED  		5
#define SBGC_SYS_SUB_ERR_INTERNAL_SYSTEM_ERROR  	6
#define SBGC_SYS_SUB_ERR_ENCODER_CALIB_BAD_SCALE  	7
#define SBGC_SYS_SUB_ERR_OVER_TEMPERATURE 			8
#define SBGC_SYS_SUB_ERR_BAD_MOTOR_POLES_INVERT 	9
#define SBGC_SYS_SUB_ERR_NOT_ENOUGH_MEMORY 			10
#define SBGC_SYS_SUB_ERR_IMU_SENSOR_NOT_RESPONDING 	11
#define SBGC_SYS_SUB_ERR_MOTOR_OVERHEAT_PROTECTION 	13
#define SBGC_SYS_SUB_ERR_MOTOR_IS_LOCKED 			14
#define SBGC_SYS_SUB_ERR_BAD_IMU_HEALTH 			15
#define SBGC_SYS_SUB_ERR_INFINITE_RESET 			16
#define SBGC_SYS_SUB_ERR_WRONG_INITIAL_POSITION 	17

// Trigger pins
#define SBGC_PIN_AUX1 		16
#define SBGC_PIN_AUX2 		17
#define SBGC_PIN_AUX3 		18
#define SBGC_PIN_BUZZER 	32
#define CMD_PIN_SSAT_POWER 	33  // pin that control Spektrum Satellite 3.3V power line (low state enables line)

// Value passed in CMD_SERVO_OUT to disable servo output
#define SBGC_SERVO_OUT_DISABLED -1


// Menu actions (used in the SBGC_MENU_BUTTON_PRESS command, menu button assignment, RC_CMD channel assignment)
#define SBGC_MENU_PROFILE1 			1
#define SBGC_MENU_PROFILE2 			2
#define SBGC_MENU_PROFILE3 			3
#define SBGC_MENU_SWAP_PITCH_ROLL 	4
#define SBGC_MENU_SWAP_YAW_ROLL 	5
#define SBGC_MENU_CALIB_ACC 		6
#define SBGC_MENU_RESET 			7
#define SBGC_MENU_SET_ANGLE 		8
#define SBGC_MENU_CALIB_GYRO 		9
#define SBGC_MENU_MOTOR_TOGGLE 		10
#define SBGC_MENU_MOTOR_ON 			11
#define SBGC_MENU_MOTOR_OFF 		12
#define SBGC_MENU_FRAME_UPSIDE_DOWN 13
#define SBGC_MENU_PROFILE4 			14
#define SBGC_MENU_PROFILE5 			15
#define SBGC_MENU_AUTO_PID 			16
#define SBGC_MENU_LOOK_DOWN 		17
#define SBGC_MENU_HOME_POSITION 	18
#define SBGC_MENU_RC_BIND 			19
#define SBGC_MENU_CALIB_GYRO_TEMP 	20
#define SBGC_MENU_CALIB_ACC_TEMP 	21
#define SBGC_MENU_BUTTON_PRESS 		22
#define SBGC_MENU_RUN_SCRIPT1 		23
#define SBGC_MENU_RUN_SCRIPT2 		24
#define SBGC_MENU_RUN_SCRIPT3 		25
#define SBGC_MENU_RUN_SCRIPT4 		26
#define SBGC_MENU_RUN_SCRIPT5 		27
#define SBGC_MENU_RUN_SCRIPT6 		28
#define SBGC_MENU_RUN_SCRIPT7 		29
#define SBGC_MENU_RUN_SCRIPT8 		30
#define SBGC_MENU_RUN_SCRIPT9 		31
#define SBGC_MENU_RUN_SCRIPT10 		32
#define SBGC_MENU_CALIB_MAG 		33
#define SBGC_MENU_LEVEL_ROLL_PITCH 	34
#define SBGC_MENU_CENTER_YAW 		35
#define SBGC_MENU_UNTWIST_CABLES 	36
#define SBGC_MENU_SET_ANGLE_NO_SAVE 37
#define SBGC_MENU_HOME_POSITION_SHORTEST 	38
#define SBGC_MENU_CENTER_YAW_SHORTEST 		39


// Control modes
#define SBGC_CONTROL_MODE_NO          0
#define SBGC_CONTROL_MODE_SPEED       1
#define SBGC_CONTROL_MODE_ANGLE       2
#define SBGC_CONTROL_MODE_SPEED_ANGLE 3
#define SBGC_CONTROL_MODE_RC          4
#define SBGC_CONTROL_MODE_ANGLE_REL_FRAME 5

#define SBGC_CONTROL_MODE_MASK 0x0F // bits0..3 used for mode, other for flags

#define SBGC_CONTROL_MODE_FLAG_UNTWIST (1<<7)


#endif /* _SBGC_ */
