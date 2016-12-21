// Header file to interface with the BaseCam SimpleBGC Controller
// Provides the commands relevant for MotoCrane in order to control the speed/acceleration of the
// Gimbal motors as well as read the real time angles.

// Sections of code taken and modified from the Alex Mos GitHub repo


// Size of header and checksums
#define SBGC_CMD_NON_PAYLOAD_BYTES 5
// Max. size of a command after packing to bytes
#define SBGC_CMD_MAX_BYTES 255
// Max. size of a payload data
#define SBGC_CMD_DATA_SIZE (SBGC_CMD_MAX_BYTES - SBGC_CMD_NON_PAYLOAD_BYTES)


////////////////////// Command start bit ////////////////
#define SBGC_CMD_START_BYTE '>'

////////////////////// Command ID definitions ////////////////
#define SBGC_CMD_READ_PARAMS  82
#define SBGC_CMD_WRITE_PARAMS  87
#define SBGC_CMD_REALTIME_DATA  68
#define SBGC_CMD_BOARD_INFO  86
#define SBGC_CMD_CALIB_ACC  65
#define SBGC_CMD_CALIB_GYRO  103
#define SBGC_CMD_CALIB_EXT_GAIN  71
#define SBGC_CMD_USE_DEFAULTS  70
#define SBGC_CMD_CALIB_POLES  80
#define SBGC_CMD_RESET  114
#define SBGC_CMD_HELPER_DATA 72
#define SBGC_CMD_CALIB_OFFSET  79
#define SBGC_CMD_CALIB_BAT  66
#define SBGC_CMD_MOTORS_ON   77
#define SBGC_CMD_MOTORS_OFF  109
#define SBGC_CMD_CONTROL   67
#define SBGC_CMD_TRIGGER_PIN  84
#define SBGC_CMD_EXECUTE_MENU 69
#define SBGC_CMD_GET_ANGLES  73
#define SBGC_CMD_CONFIRM  67

// Control modes
#define SBGC_CONTROL_MODE_NO          0
#define SBGC_CONTROL_MODE_SPEED       1
#define SBGC_CONTROL_MODE_ANGLE       2
#define SBGC_CONTROL_MODE_SPEED_ANGLE 3
#define SBGC_CONTROL_MODE_RC          4
#define SBGC_CONTROL_MODE_ANGLE_REL_FRAME 5

#define SBGC_CONTROL_MODE_MASK 0x0F // bits0..3 used for mode, other for flags

#define SBGC_CONTROL_MODE_FLAG_UNTWIST (1<<7)
