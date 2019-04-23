#ifndef _SBGC_COMMANDHELPERS_H_
#define _SBGC_COMMANDHELPERS_H_

#include "SBGC_Parser.h"


//////////////// Units conversion /////////////////
#define SBGC_ANGLE_FULL_TURN 16384
// Conversion from degree/sec to units that command understand
#define SBGC_SPEED_SCALE  (1.0f/0.1220740379f)
#define SBGC_DEGREE_ANGLE_SCALE ((float)SBGC_ANGLE_FULL_TURN/360.0f)
#define SBGC_ANGLE_DEGREE_SCALE (360.0f/(float)SBGC_ANGLE_FULL_TURN)

// Conversions for angle in degrees to angle in SBGC 14bit representation, and back
#define SBGC_DEGREE_TO_ANGLE(val) ((val)*SBGC_DEGREE_ANGLE_SCALE)
#define SBGC_ANGLE_TO_DEGREE(val) ((val)*SBGC_ANGLE_DEGREE_SCALE)
// The same, optimized for integers
#define SBGC_DEGREE_TO_ANGLE_INT(val) ((int32_t)(val)*SBGC_ANGLE_FULL_TURN/360)
#define SBGC_DEGREE_01_TO_ANGLE_INT(val) ((int32_t)(val)*SBGC_ANGLE_FULL_TURN/3600)
#define SBGC_ANGLE_TO_DEGREE_INT(val) ((int32_t)(val)*360/SBGC_ANGLE_FULL_TURN)
#define SBGC_ANGLE_TO_DEGREE_01_INT(val) ((int32_t)(val)*3600/SBGC_ANGLE_FULL_TURN)

#define ROLL	0
#define PITCH	1
#define YAW		2


// CMD_CONTROL
typedef struct{
	uint8_t mode;

	int16_t speedRoll;
	int16_t angleRoll;
	int16_t speedPitch;
	int16_t anglePitch;
	int16_t speedYaw;
	int16_t angleYaw;
}SBGC_CMD_Control_t;

// CMD_CONTROL (extended version)
typedef struct{
	uint8_t mode[3];
	struct{
		int16_t speed;
		int16_t angle;
	}data[3];
}SBGC_CMD_ControlExt_t;



//uint8_t SBGC_cmd_control_send(SBGC_CMD_Control_t *ctrl, SBGC_Parser_t *parser);
//uint8_t SBGC_cmd_control_ext_send(SBGC_CMD_ControlExt_t *ctrlExt, SBGC_Parser_t *parser);




// Realtime data unpacking functions

// CMD_REALTIME_DATA_3
typedef struct{
	struct{
		int16_t acc_data; 	// Data from the accelerometer sensor with the calibrations	applied. Units: 1/512 G
		int16_t gyro_data;	// Data from the gyroscope sensor with the calibrations applied. Units: 0,06103701895 degree/sec
	}sensor_data[3];		// axes: ROLL, PITCH, YAW
	uint16_t serial_err_cnt;
	uint16_t system_error;		// System error flags that controller may set
	uint8_t system_sub_error; 	// Specifies the reason of emergency stop
	uint8_t reserved1[3];
	int16_t rc_raw_data[SBGC_RC_NUM_CHANNELS]; // RC signal in 1000..2000 range for ROLL, PITCH, YAW, CMD, EXT_ROLL, EXT_PITCH channels
	int16_t imu_angle[3];	// imu angles of a camera in 14-bit resolution per full turn. Units: 0,02197265625 degree
	int16_t frame_imu_angle[3]; // Angles measured by the second IMU (if present), in 14-bit resolution. Units: 0,02197265625 degree
	int16_t target_angles[3]; // Target angles, in 14-bit resolution. Units: 0,02197265625 degree
	uint16_t cycle_time;	// Units: microseconds
	uint16_t i2c_error_count; // Number of registered errors on I2C bus
	uint8_t error_code;	// deprecated, replaced by the SYSTEM_ERROR variable
	uint16_t bat_level; // Battery voltage. Units: 0.01 volt
	uint8_t rt_data_flags; // bit0 set - motors are turned ON
	uint8_t cur_imu; // Currently selected IMU that provides angles and raw sensor data. IMU_TYPE_MAIN=1, IMU_TYPE_FRAME=2
	uint8_t cur_profile; // Currently selected profile
	uint8_t motor_power[3]; // actual motor power for ROLL, PITCH, YAW axis, 0..255
}SBGC_RealtimeData_3_t;

// CMD_REALTIME_DATA_4
typedef struct{
	SBGC_RealtimeData_3_t cmd_realtime_data_3;

	int16_t stator_rotor_angle[3]; // relative angle of each motor, 16384/360 degrees
	uint8_t reserved2;
	int16_t balance_error[3]; // Error in balance (0 – perfect balance, 512 - 100% of the motor power is required to hold a camera)
	uint16_t current; // Actual current consumption. Units: mA
	int16_t mag_data[3]; // Raw data from magnetometer. Units: relative, calibrated for current environment to give ±1000 for each axis.
	int8_t imu_temperature;	// Temperature of IMU sensors. Units: Celsius
	int8_t frame_imu_temperature; // Temperature of frame IMU sensors. Units: Celsius
	uint8_t imu_g_err; //Error between estimated gravity vector and reference vector for currently active IMU. Units: 0.1 degree
	uint8_t imu_h_err; // Error between estimated heading vector and reference vector for currently active IMU. Units: 0.1 degree
	int16_t motor_out[3]; // Motor effective output, proportional to torque. Max. value of ±10000 equals to applying full power
	uint8_t reserved3[30];
}SBGC_RealtimeData_4_t;


uint8_t SBGC_cmd_realtime_data_3_unpack(SBGC_RealtimeData_3_t *rtData3, SBGC_SerialCommand_t *cmd);
uint8_t SBGC_cmd_realtime_data_4_unpack(SBGC_RealtimeData_4_t *rtData4, SBGC_SerialCommand_t *cmd);

#endif /* _SBGC_COMMANDHELPERS_H_ */
