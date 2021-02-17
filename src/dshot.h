#ifndef _DSHOT_H
#define _DSHOT_H

#include <Arduino.h>
#include "stdint.h"
#include "DMAChannel.h"

#define F_TMR F_BUS_ACTUAL

#define DSHOT_MAX_OUTPUT          4             // Total number of DSHOT outputs configured (more may be configurable)

#define DSHOT_DMA_LENGTH          18            // Number of steps of one DMA sequence (the two last values are zero)
#define DSHOT_DMA_MARGIN          2             // Number of additional bit duration to wait until checking if DMA is over
#define DSHOT_LENGTH              16            // Number of bits in a DSHOT sequence
#define DSHOT_BIT_DURATION        1670          // Duration of 1 DSHOT600 bit in ns
#define DSHOT_1B_DURATION         1250          // Duration of a DSHOT600 long pulse in ns
#define DSHOT_0B_DURATION         625           // Duration of a DSHOT600 short pulse in ns
#define DSHOT_MAX_VALUE           2047          // Maximum DSHOT value

#define DSHOT_TLM_LENGTH          10            // Number of bytes in the telemetry packet

// Borrowed from betaflight pwm_output.h
typedef enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO,                       // V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,                    // 9
    DSHOT_CMD_3D_MODE_ON,                     // 10
    DSHOT_CMD_SETTINGS_REQUEST,               // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,                  // 12
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON,                        // BLHeli32 only
    DSHOT_CMD_LED1_ON,                        // BLHeli32 only
    DSHOT_CMD_LED2_ON,                        // BLHeli32 only
    DSHOT_CMD_LED3_ON,                        // BLHeli32 only
    DSHOT_CMD_LED0_OFF,                       // BLHeli32 only
    DSHOT_CMD_LED1_OFF,                       // BLHeli32 only
    DSHOT_CMD_LED2_OFF,                       // BLHeli32 only
    DSHOT_CMD_LED3_OFF,                       // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30,  // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31,        // KISS silent Mode on/Off
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
    DSHOT_CMD_MAX = 47
} DSHOT_command;

typedef struct {
    uint8_t temp; 
    uint16_t voltage;
    uint16_t amps;
    uint16_t ampHours;
    uint16_t rpm;
} DSHOT_telemetry;

enum{
    DSHOT_TLM_NONE = 0,
    DSHOT_TLM_REQUEST,
};

#define DSHOT_ARMING_REPS       50 
#define DSHOT_SETTING_REPS      10 
#define DSHOT_COMMAND_DELAY_US 1000

// Used to extract ESC_INFO from TLM port
#define ESC_INFO_KISS_V1_EXPECTED_FRAME_SIZE 15
#define ESC_INFO_KISS_V2_EXPECTED_FRAME_SIZE 21
#define ESC_INFO_BLHELI32_EXPECTED_FRAME_SIZE 64
#define ESC_INFO_VERSION_POSITION 12

class DShot
{
public:
    
    DSHOT_telemetry tlm_data[DSHOT_MAX_OUTPUT];

	DShot( uint8_t count );

    void setup();

    void write( uint16_t *cmd, uint8_t *tlm );
    void write( uint8_t num, uint16_t cmd, uint8_t tlm );

    void armMotor( uint8_t num );
    void armMotors( void );

    void setConfig( uint8_t num, DSHOT_command param );

    void saveConfig( uint8_t num );

    void requestConfig( uint8_t num, Stream * port );

    DSHOT_telemetry *readTelemetry( uint8_t num, Stream * port );

private:

    // Number of configured outputs
    uint8_t output_count;

    // DMA output buffer
    volatile uint16_t buffer[DSHOT_MAX_OUTPUT][DSHOT_DMA_LENGTH];

    uint8_t tlm_buffer[DSHOT_MAX_OUTPUT][DSHOT_TLM_LENGTH];

    // Output buffers to temporary hold command and telemetry requests
    uint8_t tlm[DSHOT_MAX_OUTPUT];
    uint16_t cmd[DSHOT_MAX_OUTPUT];

    // Telemetry CRC
	uint8_t update_crc8(uint8_t crc, uint8_t crcSeed);
	uint8_t get_crc8(uint8_t * buf, uint8_t bufLen);

};

#endif
