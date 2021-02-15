#include "dshot.h"

// PWM output pins choosen for DSHOT
uint8_t DSHOT_pin[DSHOT_NUM_PORT] = {
    4, 
    8, 
    24, 
    22, 
};

// Pointers to the eFlexPWM modules choosen to generate the DSHOT outputs
IMXRT_FLEXPWM_t *DSHOT_mod[DSHOT_NUM_PORT]  = { 
    &IMXRT_FLEXPWM2, 
    &IMXRT_FLEXPWM1,  
    &IMXRT_FLEXPWM1,  
    &IMXRT_FLEXPWM4,
};

// Index of the sub_module used from the eFlexPWM modules
uint8_t DSHOT_sub[DSHOT_NUM_PORT] = {
    0, 
    3, 
    2, 
    0, 
};

// Channel selector for each sub_module used. 0=A, 1=B
uint8_t DSHOT_cha[DSHOT_NUM_PORT] = {
    0, 
    0, 
    2, 
    0,
};   

// ALT value used to map/mux the PWM output to the physical output pin.
uint8_t DSHOT_pinmux[DSHOT_NUM_PORT] = {
    1, 
    6, 
    4, 
    1, 
};

// Hardware events used to trigger next DMA transfer. Triggers at end of each PWM write
uint8_t DSHOT_event[DSHOT_NUM_PORT] = {
    DMAMUX_SOURCE_FLEXPWM2_WRITE0,
    DMAMUX_SOURCE_FLEXPWM1_WRITE3,
    DMAMUX_SOURCE_FLEXPWM1_WRITE2,
    DMAMUX_SOURCE_FLEXPWM4_WRITE0,
};

// DMA objects
DMAChannel DSHOT_dma[DSHOT_NUM_PORT];

// DMA termination interrupts for each DMA channel
#define DSHOT_dma_interrupt( INDEX ) \
static void DSHOT_dma_interrupt_ ## INDEX( void ) { \
    DSHOT_dma[INDEX].clearInterrupt(); \
    (*DSHOT_mod[INDEX]).MCTRL &= FLEXPWM_MCTRL_RUN( 1 << DSHOT_sub[INDEX] );  \
}

DSHOT_dma_interrupt( 0 );
DSHOT_dma_interrupt( 1 );
DSHOT_dma_interrupt( 2 );
DSHOT_dma_interrupt( 3 );

static void (*DSHOT_DMA_ISR[DSHOT_NUM_PORT])() = {
    DSHOT_dma_interrupt_0,
    DSHOT_dma_interrupt_1,
    DSHOT_dma_interrupt_2,
    DSHOT_dma_interrupt_3
};

DShot::DShot( uint8_t num ){

    /* --- Initiliaztion --- */
    // Calculate pulse durations
    short_pulse  = uint64_t(F_TMR) * DSHOT_0B_DURATION / 1000000000;     // DSHOT short pulse duration (nb of F_BUS periods)
    long_pulse   = uint64_t(F_TMR) * DSHOT_1B_DURATION / 1000000000;     // DSHOT long pulse duration (nb of F_BUS periods)
    bit_length   = uint64_t(F_TMR) * DSHOT_BIT_DURATION / 1000000000;     // DSHOT bit duration (nb of F_BUS periods)   

    // Initialize buffer with zeroes
    for ( uint8_t i = 0; i < DSHOT_LENGTH; i++ ) {
        dma_buffer[i] = 0;
    }

    // Index of the pre-selected configuration
    this->index = num;

    // Select eFlexPWM module and sub_module for PWM generation based on index
    pwm_module = DSHOT_mod[index];
    sub_module = DSHOT_sub[index];
}

void DShot::setup(){
    /* --- Configure output pin as DSHOT outputs --- */
    // The pin is configured as eFlexPWM (FLEXPWMn) PWM output
    *(portConfigRegister( DSHOT_pin[index] ))  = DSHOT_pinmux[index];

    /* --- Configuration of eFlexPWM module --- */
    // Configure eFlexPWM module. Example: IMXRT_FLEXPWM2.SM[0]...
    (*pwm_module).SM[sub_module].INIT = 0;
    (*pwm_module).SM[sub_module].VAL0 = 0;
    (*pwm_module).SM[sub_module].VAL1 = bit_length;
    (*pwm_module).SM[sub_module].VAL2 = 0;
    (*pwm_module).SM[sub_module].VAL3 = 0;
    (*pwm_module).SM[sub_module].VAL4 = 0;
    (*pwm_module).SM[sub_module].VAL5 = 0;

    // Configuration of module channel
    if ( DSHOT_cha[index] == 1 ) {
        (*pwm_module).OUTEN |= FLEXPWM_OUTEN_PWMB_EN(1 << sub_module);
    } else {
        (*pwm_module).OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << sub_module);
    }

    // DMA Enable Register. VALDE bit enables DMA write requests from VALx and FRACVALx registers
    (*pwm_module).SM[sub_module].DMAEN = FLEXPWM_SMDMAEN_VALDE;


    /* --- Configuration of DMA transfer --- */
    DSHOT_dma[index].sourceBuffer( dma_buffer, DSHOT_DMA_LENGTH * sizeof( uint16_t ) );
    // Depending on the channel, write to different destination
    if ( DSHOT_cha[index] == 1 ) {
        DSHOT_dma[index].destination( (uint16_t&) (*pwm_module).SM[sub_module].VAL5 );
    } else {
        DSHOT_dma[index].destination( (uint16_t&) (*pwm_module).SM[sub_module].VAL3 );
    }
    DSHOT_dma[index].triggerAtHardwareEvent( DSHOT_event[index] );
    DSHOT_dma[index].interruptAtCompletion();
    DSHOT_dma[index].attachInterrupt( DSHOT_DMA_ISR[index] );
    DSHOT_dma[index].enable(); 

}

void DShot::write( uint16_t cmd, bool tlm ){
    uint16_t data;

    // Compute the packet to send
    // 11 first MSB = command
    // 12th MSB = telemetry request
    // 4 LSB = CRC
    data = ( cmd << 5 ) | ( (uint8_t)tlm << 4 );
    data |= ( ( data >> 4 ) ^ ( data >> 8 ) ^ ( data >> 12 ) ) & 0x0f;

    // Generate and write DSHOT timings to DMA buffer
    for ( uint8_t i = 0; i < DSHOT_LENGTH; i++ )  {
        if ( data & ( 1 << ( DSHOT_LENGTH - 1 - i ) ) ) {
            dma_buffer[i] = long_pulse;
        } else {
            dma_buffer[i] = short_pulse;
        }
    }

    // Enable PWM generator in the corresponding submodule.
    (*pwm_module).MCTRL |= FLEXPWM_MCTRL_RUN( 1 << sub_module ); 

}
