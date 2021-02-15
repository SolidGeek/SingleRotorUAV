#ifndef _DSHOT_H
#define _DSHOT_H

#include <Arduino.h>
#include "stdint.h"
#include "DMAChannel.h"

#define F_TMR F_BUS_ACTUAL

// Total number of DSHOT outputs configured
#define DSHOT_NUM_PORT          4

#define DSHOT_DMA_LENGTH          18            // Number of steps of one DMA sequence (the two last values are zero)
#define DSHOT_DMA_MARGIN          2             // Number of additional bit duration to wait until checking if DMA is over
#define DSHOT_LENGTH              16            // Number of bits in a DSHOT sequence
#define DSHOT_BIT_DURATION        1670          // Duration of 1 DSHOT600 bit in ns
#define DSHOT_1B_DURATION         1250          // Duration of a DSHOT600 long pulse in ns
#define DSHOT_0B_DURATION         625           // Duration of a DSHOT600 short pulse in ns
#define DSHOT_MAX_VALUE           2047          // Maximum DSHOT value


class DShot
{
public:

	struct Telemetry{
		float temp; 
		float voltage;
		float amps;
		float ampHours;
		float rpm;
	} tlm;

	DShot( uint8_t num );

  void setup();
 
  void write( uint16_t cmd, bool tlm );

private:

    // Index of the choosen preconfigured module
    uint8_t index;

	  DMAChannel *dma;
    IMXRT_FLEXPWM_t *pwm_module;
    uint8_t sub_module;

    // DMA buffer
    volatile uint16_t dma_buffer[DSHOT_DMA_LENGTH];
    
    uint16_t short_pulse, long_pulse, bit_length;

};

#endif
