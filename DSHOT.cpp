/*
 *  DSHOT:    Generation of up to 6 DSHOT signals using DMA
 *
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Arda Yiğit and Jacques Gangloff
 *  Date:     May 2019
 */

// Includes
#include <Arduino.h>
#include "DMAChannel.h"
#include "DSHOT.h"

/*
 *  Constants
 */

#if defined(__IMXRT1062__) // teensy 4.0
  #define F_TMR F_BUS_ACTUAL
#else // teensy 3.5
  #define F_TMR F_BUS
#endif

/* Defining DSHOT600 timings expressed in F_TMR periods
 * DSHOT600 has the following timings:
 *
 *          1670ns
 *          --------->
 *          ______
 * 1 bit :  |     |___|
 *          1250ns
 *          ____
 * 0 bit :  |   |_____|
 *          625ns
 *
 * On the teensy 3.5, F_TMR == 60000000 (60MHz)
 * On the teensy 4.0, F_TMR == 600000000 (600Mhz)
 */
const uint16_t DSHOT_short_pulse  = uint64_t(F_TMR) * DSHOT_SP_DURATION / 1000000000;     // DSHOT short pulse duration (nb of F_BUS periods)
const uint16_t DSHOT_long_pulse   = uint64_t(F_TMR) * DSHOT_LP_DURATION / 1000000000;     // DSHOT long pulse duration (nb of F_BUS periods)
const uint16_t DSHOT_bit_length   = uint64_t(F_TMR) * DSHOT_BT_DURATION / 1000000000;     // DSHOT bit duration (nb of F_BUS periods)

/*
 *  Global variables
 */

// Number of initialized DSHOT outputs
uint8_t             DSHOT_n;

#if defined(__IMXRT1062__) // teensy 4.0

// DMA eFlexPWM modules
volatile IMXRT_FLEXPWM_t  *DSHOT_mods[DSHOT_NB_DMA_CHAN]  = { &IMXRT_FLEXPWM2, 
                                                              &IMXRT_FLEXPWM1,  
                                                              &IMXRT_FLEXPWM1,  
                                                              &IMXRT_FLEXPWM4,  
                                                              &IMXRT_FLEXPWM4, 
                                                              &IMXRT_FLEXPWM2
                                                            };

// DMA eFlexPWM submodules
volatile uint8_t          DSHOT_sm[DSHOT_NB_DMA_CHAN]     = { 0, 
                                                              3, 
                                                              2, 
                                                              0, 
                                                              1, 
                                                              2
                                                            };

// DMA eFlexPWM submodule PWM channel selector: A=0, B=1, X=2
volatile uint8_t  	      DSHOT_abx[DSHOT_NB_DMA_CHAN]    = { 0, 
                                                              0, 
                                                              2, 
                                                              0, 
                                                              0, 
                                                              1
                                                            };

// Output pins
volatile uint8_t          DSHOT_pin[DSHOT_NB_DMA_CHAN]    = { 4, 
                                                              8, 
                                                              24, 
                                                              22, 
                                                              23, 
                                                              9
                                                            };

// Output pin ALT mux
volatile uint8_t          DSHOT_pinmux[DSHOT_NB_DMA_CHAN] = { 1, 
                                                              6, 
                                                              4, 
                                                              1, 
                                                              1, 
                                                              2
                                                            };

// DMA source
volatile uint8_t          DSHOT_dmamux[DSHOT_NB_DMA_CHAN] = { DMAMUX_SOURCE_FLEXPWM2_WRITE0,
                                                              DMAMUX_SOURCE_FLEXPWM1_WRITE3,
                                                              DMAMUX_SOURCE_FLEXPWM1_WRITE2,
                                                              DMAMUX_SOURCE_FLEXPWM4_WRITE0,
                                                              DMAMUX_SOURCE_FLEXPWM4_WRITE1,
                                                              DMAMUX_SOURCE_FLEXPWM2_WRITE2
                                                            };

#else // teensy 3.5

// DMA FTM channel values references
volatile uint32_t*  DSHOT_DMA_chan_teensy[DSHOT_NB_DMA_CHAN] ={   &FTM0_C0V,
                                                                  &FTM0_C1V,
                                                                  &FTM0_C4V,
                                                                  &FTM0_C5V,
                                                                  &FTM0_C6V,
                                                                  &FTM0_C7V };

// DMA FTM channel status and control register
volatile uint32_t*  DSHOT_DMA_chsc_teensy[DSHOT_NB_DMA_CHAN] ={   &FTM0_C0SC,
                                                                  &FTM0_C1SC,
                                                                  &FTM0_C4SC,
                                                                  &FTM0_C5SC,
                                                                  &FTM0_C6SC,
                                                                  &FTM0_C7SC };


// Output pins
volatile uint32_t*  DSHOT_DMA_pin_teensy[DSHOT_NB_DMA_CHAN] ={    &CORE_PIN22_CONFIG,
                                                                  &CORE_PIN23_CONFIG,
                                                                  &CORE_PIN6_CONFIG,
                                                                  &CORE_PIN20_CONFIG,
                                                                  &CORE_PIN21_CONFIG,
                                                                  &CORE_PIN5_CONFIG };

#endif

// DMA objects
DMAChannel          dma[DSHOT_MAX_OUTPUTS];

// DMA data
volatile uint16_t   DSHOT_dma_data[DSHOT_DMA_LENGTH];

/* 
 * DMA termination interrupt service routine (ISR) for each DMA channel
 */
#define DSHOT_DMA_interrupt_routine( DSHOT_CHANNEL ) \
void DSHOT_DMA_interrupt_routine_ ## DSHOT_CHANNEL( void ) { \
  dma[DSHOT_CHANNEL].clearInterrupt( ); \
  (*DSHOT_mods[DSHOT_CHANNEL]).MCTRL &= FLEXPWM_MCTRL_RUN( 0 << DSHOT_sm[DSHOT_CHANNEL] );  \
}

DSHOT_DMA_interrupt_routine( 0 );
DSHOT_DMA_interrupt_routine( 1 );
DSHOT_DMA_interrupt_routine( 2 );
DSHOT_DMA_interrupt_routine( 3 );
DSHOT_DMA_interrupt_routine( 4 );
DSHOT_DMA_interrupt_routine( 5 );

void (*DSHOT_DMA_ISR[6])()  = { DSHOT_DMA_interrupt_routine_0,
                                DSHOT_DMA_interrupt_routine_1,
                                DSHOT_DMA_interrupt_routine_2,
                                DSHOT_DMA_interrupt_routine_3,
                                DSHOT_DMA_interrupt_routine_4,
                                DSHOT_DMA_interrupt_routine_5
                              };


/*
 *  Initialize the DMA hardware in order to be able
 *  to generate 6 DSHOT outputs.
 */
void DSHOT_init( int index ) {

  // Initialize DMA data

  for ( uint8_t i = 0; i < DSHOT_DMA_LENGTH; i++ ) {
    DSHOT_dma_data[i] = 0;
  }
  

  // Configure pins on the board as DSHOT outputs
  // These pins are configured as eFlexPWM (FLEXPWMn) PWM outputs
  
  *(portConfigRegister( DSHOT_pin[index] ))  = DSHOT_pinmux[index];
 

  // Configure eFlexPWM modules and submodules for PWM generation
  // --- submodule specific registers ---
  // INIT: initial counter value
  // VAL0: PWM_X compare value
  // VAL1: counter max value
  // VAL2: must be 0 for edge-aligned PWM
  // VAL3: PWM_A compare value
  // VAL4: must be 0 for edge-aligned PWM
  // VAL5: PWM_B compare value
  // OCTRL: invert polarity of PWMq FLEXPWM_SMOCTRL_POLq
  // DMAEN: FLEXPWM_SMDMAEN_VALDE to enable DMA
  // --- module specific registers ---
  // OUTEN: output enable for submodule n and PWM q FLEXPWM_OUTEN_PWMq_EN( 1 << n )
  

  IMXRT_FLEXPWM_t * pwm_module = DSHOT_mods[index];
  uint16_t sub_module = DSHOT_sm[index];
  
  (*pwm_module).SM[sub_module].INIT = 0;
  (*pwm_module).SM[sub_module].VAL0 = 0;
  (*pwm_module).SM[sub_module].VAL1 = DSHOT_bit_length;
  (*pwm_module).SM[sub_module].VAL2 = 0;
  (*pwm_module).SM[sub_module].VAL3 = 0;
  (*pwm_module).SM[sub_module].VAL4 = 0;
  (*pwm_module).SM[sub_module].VAL5 = 0;
  
  if ( DSHOT_abx[index] == 1 ) {
    (*pwm_module).OUTEN |= FLEXPWM_OUTEN_PWMB_EN(1 << sub_module);
  } else {
    (*pwm_module).OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << sub_module);
  }
  (*pwm_module).SM[sub_module].DMAEN = FLEXPWM_SMDMAEN_VALDE;
 

  // Each DMA channel is linked to a unique eFlexPWM submodule
  // DMA channels are triggered by independant hardware events

  dma[index].sourceBuffer( DSHOT_dma_data, DSHOT_DMA_LENGTH * sizeof( uint16_t ) );
  if ( DSHOT_abx[index] == 1 ) {
    dma[index].destination( (uint16_t&) (*pwm_module).SM[sub_module].VAL5 );
  } else {
    dma[index].destination( (uint16_t&) (*pwm_module).SM[sub_module].VAL3 );
  }
  dma[index].triggerAtHardwareEvent( DSHOT_dmamux[index] );
  dma[index].interruptAtCompletion( );
  // dma[index].transferSize(1);
  // dma[index].transferCount(DSHOT_DMA_LENGTH);
  // dma[index].disableOnCompletion();
  dma[index].attachInterrupt( DSHOT_DMA_ISR[index] );
  dma[index].enable( );

}

//
//  Send the DSHOT signal through all the configured channels
//  "cmd" points to the DSHOT_MAX_OUTPUTS DSHOT commands to send
//  Telemetry is requested with "tlm", CRC bits are added
//
//  Returns an error code in case of failure, 0 otherwise:
//
int DSHOT_send( uint16_t cmd, uint8_t tlm ) {

  uint16_t  data;
  memset(DSHOT_dma_data, 0, DSHOT_DMA_LENGTH);

  // Check cmd value
  if ( cmd > DSHOT_MAX_VALUE ) {
    return DSHOT_ERROR_RANGE;
  }

  // Compute the packet to send
  // 11 first MSB = command
  // 12th MSB = telemetry request
  // 4 LSB = CRC
  data = ( cmd << 5 ) | ( tlm << 4 );
  data |= ( ( data >> 4 ) ^ ( data >> 8 ) ^ ( data >> 12 ) ) & 0x0f;

  // Generate DSHOT timings corresponding to the packet
  for ( uint8_t i = 0; i < DSHOT_DSHOT_LENGTH; i++ )  {
    if ( data & ( 1 << ( DSHOT_DSHOT_LENGTH - 1 - i ) ) ) {
      DSHOT_dma_data[i] = DSHOT_long_pulse;
    } else {
      DSHOT_dma_data[i] = DSHOT_short_pulse;
    }
  }
  

  // Start DMA by activating the clocks
  // Clocks are disabled again by the DMA ISRs
  (*DSHOT_mods[0]).MCTRL |= FLEXPWM_MCTRL_RUN( 1 << DSHOT_sm[0] ); 

  // Wait the theoretical time needed by DMA + some margin
  // delayMicroseconds( (unsigned int)( ( DSHOT_BT_DURATION * ( DSHOT_DMA_LENGTH + DSHOT_DMA_MARGIN ) ) / 1000 ) );

  return 0;
}
