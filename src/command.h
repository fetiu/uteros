#pragma once

typedef enum {
	COMMAND_ID_SETTING,
	COMMAND_ID_CONFIRM,
	COMMAND_OP_START,
	COMMAND_OP_STOP,
	COMMAND_OP_PAUSE_RESUME,
	COMMAND_OP_POWER_OFF,
	COMMAND_SET_HEATER,
	COMMAND_SET_MOTOR,
	COMMAND_SET_MEASURE,
	COMMAND_SET_PATTERN,
	COMMAND_SET_LEVEL,
	COMMAND_MEASURED_DATA,
	MAX_COMMAND_TYPE
} CommandType;
