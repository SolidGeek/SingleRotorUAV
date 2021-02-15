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

class DShot
{
public:

	struct Telemetry{
		float temp; 
		float voltage;
		float amps;
		float ampHours;
		float rpm;
	} tlm[DSHOT_MAX_OUTPUT];

	DShot( uint8_t count );

    void setup();

    void write( uint16_t *cmd, uint8_t *tlm );
    void write( uint8_t num, uint16_t cmd, uint8_t tlm );

private:

    // Number of configured outputs
    uint8_t output_count;

    // DSHOT output buffer
    volatile uint16_t buffer[DSHOT_MAX_OUTPUT][DSHOT_DMA_LENGTH];

    uint8_t tlm_buffer[DSHOT_MAX_OUTPUT][DSHOT_TLM_LENGTH];

};

#endif
