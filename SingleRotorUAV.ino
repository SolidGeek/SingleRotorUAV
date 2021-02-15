#include "DSHOT.h"

#define ESCCMD_CMD_REPETITION   10                // Number of time commands have to be repeated to be acknowledged by ESC
#define ESCCMD_CMD_ARMING_REP   25                // Number of command repetition to arm
#define ESCCMD_CMD_DELAY        50                // Delay between two consecutive DSHOT transmissions (us)

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
} DSHOT_commands;

void setup() {

  Serial.begin(9600);
  Serial.println("HEY");
  // DSHOT_init(4);

  // Send command ESCCMD_CMD_ARMING_REP times
  /* for ( int i = 0; i < ESCCMD_CMD_ARMING_REP; i++ )  {

    // Send DSHOT signal to all ESCs
    DSHOT_send( (uint16_t)DSHOT_CMD_MOTOR_STOP, (uint8_t)0 );
    // Wait some time
    delayMicroseconds( 2 * ESCCMD_CMD_DELAY );
  }*/ 

  delay(100);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  // DSHOT_send( (uint16_t)200, (uint8_t)1 );
  Serial.println("HELLO");
  delay(100);
}
