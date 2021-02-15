#include <Arduino.h>
#include "DMAChannel.h"
#include "dshot.h"

HardwareSerial*  DSHOT_serial[DSHOT_MAX_OUTPUT] = {
    &Serial1,
    &Serial2,
    &Serial3,
    &Serial4
};

// Calculate pulse durations
const uint16_t DSHOT_short_pulse  = uint64_t(F_TMR) * DSHOT_0B_DURATION / 1000000000;     // DSHOT short pulse duration (nb of F_BUS periods)
const uint16_t DSHOT_long_pulse   = uint64_t(F_TMR) * DSHOT_1B_DURATION / 1000000000;     // DSHOT long pulse duration (nb of F_BUS periods)
const uint16_t DSHOT_bit_length   = uint64_t(F_TMR) * DSHOT_BIT_DURATION / 1000000000;     // DSHOT bit duration (nb of F_BUS periods)  

// Pointers to the eFlexPWM modules choosen to generate the DSHOT outputs
IMXRT_FLEXPWM_t *DSHOT_mod[DSHOT_MAX_OUTPUT]  = { 
    &IMXRT_FLEXPWM1,
    &IMXRT_FLEXPWM2,
    &IMXRT_FLEXPWM4,
    &IMXRT_FLEXPWM4, 
};

// Index of the sub_module used from the eFlexPWM modules
uint8_t DSHOT_sub[DSHOT_MAX_OUTPUT] = {
    3,
    2,
    0,
    1
};

// Channel selector for each sub_module used. 0=A, 1=B, 2=X
uint8_t DSHOT_cha[DSHOT_MAX_OUTPUT] = {
    0, 
    1,
    0,
    0
};   

// PWM output pins choosen for DSHOT
uint8_t DSHOT_pin[DSHOT_MAX_OUTPUT] = {
    8, 
    9,
    22, 
    23, 
};

// ALT value used to map/mux the PWM output to the physical output pin.
uint8_t DSHOT_pinmux[DSHOT_MAX_OUTPUT] = {
    6,
    2,
    1,
    1
};

// Hardware events used to trigger next DMA transfer. Triggers at end of each PWM write
uint8_t DSHOT_event[DSHOT_MAX_OUTPUT] = {
    DMAMUX_SOURCE_FLEXPWM1_WRITE3,
    DMAMUX_SOURCE_FLEXPWM2_WRITE2,
    DMAMUX_SOURCE_FLEXPWM4_WRITE0,
    DMAMUX_SOURCE_FLEXPWM4_WRITE1
};

// DMA objects
DMAChannel DSHOT_dma[DSHOT_MAX_OUTPUT];

// DMA termination interrupts for each DMA channel
#define DSHOT_dma_interrupt( INDEX ) \
static void DSHOT_dma_interrupt_ ## INDEX( void ) { \
    DSHOT_dma[INDEX].clearInterrupt(); \
    (*DSHOT_mod[INDEX]).MCTRL &= FLEXPWM_MCTRL_RUN( 0 << DSHOT_sub[INDEX] );  \
}

DSHOT_dma_interrupt( 0 );
DSHOT_dma_interrupt( 1 );
DSHOT_dma_interrupt( 2 );
DSHOT_dma_interrupt( 3 );

static void (*DSHOT_DMA_ISR[DSHOT_MAX_OUTPUT])() = {
    DSHOT_dma_interrupt_0,
    DSHOT_dma_interrupt_1,
    DSHOT_dma_interrupt_2,
    DSHOT_dma_interrupt_3
};

DShot::DShot( uint8_t count ){
    /* --- Initiliaztion --- */
    // Index of the pre-selected configuration
    this->output_count = count;

    // Initialize buffers with zeroes
    for (uint8_t n = 0; n < output_count; n++){
        for ( uint8_t i = 0; i < DSHOT_LENGTH; i++ ) {
            buffer[n][i] = 0;
        }
    }
}

void DShot::setup(){
    // Configure the selected amount of output channels
    for (uint8_t n = 0; n < output_count; n++)
    {
        // Select eFlexPWM module and sub_module for PWM generation based on index
        IMXRT_FLEXPWM_t *pwm_module = DSHOT_mod[n];
        uint8_t sub_module = DSHOT_sub[n];

        /* --- Configure output pin as DSHOT outputs --- */
        // The pin is configured as eFlexPWM (FLEXPWMn) PWM output
        *(portConfigRegister( DSHOT_pin[n] ))  = DSHOT_pinmux[n];

        /* --- Configuration of eFlexPWM module --- */
        // Configure eFlexPWM module. Example: IMXRT_FLEXPWM2.SM[0]...
        (*pwm_module).SM[sub_module].INIT = 0;
        (*pwm_module).SM[sub_module].VAL0 = 0;
        (*pwm_module).SM[sub_module].VAL1 = DSHOT_bit_length;
        (*pwm_module).SM[sub_module].VAL2 = 0;
        (*pwm_module).SM[sub_module].VAL3 = 0;
        (*pwm_module).SM[sub_module].VAL4 = 0;
        (*pwm_module).SM[sub_module].VAL5 = 0;

        // Configuration of module channel
        if( DSHOT_cha[n] == 2){
            (*pwm_module).SM[sub_module].OCTRL = FLEXPWM_SMOCTRL_POLX; // invert polarity of PWM
            (*pwm_module).OUTEN |= FLEXPWM_OUTEN_PWMX_EN(1 << sub_module);
        } else if ( DSHOT_cha[n] == 1 ) {
            (*pwm_module).OUTEN |= FLEXPWM_OUTEN_PWMB_EN(1 << sub_module);
        } else {
            (*pwm_module).OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << sub_module);
        }

        // DMA Enable Register. VALDE bit enables DMA write requests from VALx and FRACVALx registers
        (*pwm_module).SM[sub_module].DMAEN = FLEXPWM_SMDMAEN_VALDE;


        /* --- Configuration of DMA transfer --- */
        DSHOT_dma[n].sourceBuffer( buffer[n], DSHOT_DMA_LENGTH * sizeof( uint16_t ) );
        // Depending on the channel, write to different destination
        if( DSHOT_cha[n] == 2 ){
            DSHOT_dma[n].destination( (uint16_t&) (*pwm_module).SM[sub_module].VAL0 );
        } else if ( DSHOT_cha[n] == 1 ) {
            DSHOT_dma[n].destination( (uint16_t&) (*pwm_module).SM[sub_module].VAL5 );
        } else {
            DSHOT_dma[n].destination( (uint16_t&) (*pwm_module).SM[sub_module].VAL3 );
        }

        DSHOT_dma[n].triggerAtHardwareEvent( DSHOT_event[n] );
        DSHOT_dma[n].interruptAtCompletion();
        DSHOT_dma[n].attachInterrupt( DSHOT_DMA_ISR[n] );
        DSHOT_dma[n].enable(); 
    }

}

void DShot::write( uint8_t num, uint16_t cmd, uint8_t tlm ){
    uint16_t data;

    // Compute the packet to send
    // 11 first MSB = command
    // 12th MSB = telemetry request
    // 4 LSB = CRC
    data = ( cmd << 5 ) | ( tlm << 4 );
    data |= ( ( data >> 4 ) ^ ( data >> 8 ) ^ ( data >> 12 ) ) & 0x0f;

    // Generate and write DSHOT timings to DMA buffer
    for ( uint8_t i = 0; i < DSHOT_LENGTH; i++ )  {
        if ( data & ( 1 << ( DSHOT_LENGTH - 1 - i ) ) ) {
            buffer[num][i] = DSHOT_long_pulse;
        } else {
            buffer[num][i] = DSHOT_short_pulse;
        }
    }

    // Enable PWM generator in the corresponding submodule (clocks out the newly added data).
    (*DSHOT_mod[num]).MCTRL |= FLEXPWM_MCTRL_RUN( 1 << DSHOT_sub[num] ); 
}


void DShot::write( uint16_t *cmd, uint8_t *tlm ){
    uint16_t data;

    // For each output
    for (uint8_t n = 0; n < output_count; n++)
    {
        // Compute the packet to send
        // 11 first MSB = command
        // 12th MSB = telemetry request
        // 4 LSB = CRC
        data = ( cmd[n] << 5 ) | ( tlm[n] << 4 );
        data |= ( ( data >> 4 ) ^ ( data >> 8 ) ^ ( data >> 12 ) ) & 0x0f;

        // Generate and write DSHOT timings to DMA buffer
        for ( uint8_t i = 0; i < DSHOT_LENGTH; i++ )  {
            if ( data & ( 1 << ( DSHOT_LENGTH - 1 - i ) ) ) {
                buffer[n][i] = DSHOT_long_pulse;
            } else {
                buffer[n][i] = DSHOT_short_pulse;
            }
        }
    }
    
    // Enable PWM generator for n submodules at the same time (clocks out the newly added data).
    for (uint8_t n = 0; n < output_count; n++)
    {
        (*DSHOT_mod[n]).MCTRL |= FLEXPWM_MCTRL_RUN( 1 << DSHOT_sub[n] ); 
    }
}

/* 
DShot::parse_tlm(){

    for (uint8_t n = 0; n < output_count; n++)
    {
        // Process all available telemetry packets
        while ( read_packet( i ) )  {

        	uint8_t valid = get_crc8(buf, 9); 
      
            if(valid == buf[9]){

                // Telemetry success, CRC was valid 
                tlm.temp       	= (float)(buf[0]);
                tlm.voltage    	= (float)((buf[1]<<8)|buf[2]) / 100.0;
                tlm.amps       	= (float)((buf[3]<<8)|buf[4]) / 100.0;
                tlm.ampHours   	= (float)((buf[5]<<8)|buf[6]);
                tlm.rpm        	= (float)((buf[7]<<8)|buf[8]) * 100.0 / 7.0; 
                tlm.timestamp 	= micros();
                return true;
            }
        }
    }
}

DShot::read_packet( uint8_t num ){
    // Read all bytes in rx buffer up to packet length
    while ( ( ESCCMD_serial[i]->available( ) ) &&  ( buffer_idx[i] < ESCCMD_TLM_LENGTH ) ) { 
        serial_ret = ESCCMD_serial[i]->read( );
        
        if ( serial_ret >= 0 )  {
            ESCCMD_bufferTlm[i][buffer_idx[i]] = (uint8_t)serial_ret;
            buffer_idx[i]++;
        }
    }

    // Check if a complete packet has arrived
    if ( buffer_idx[i] == ESCCMD_TLM_LENGTH  )  {
        // Reset byte index in packet buffer
        buffer_idx[i] = 0;

        // Return pointer to buffer
        return ESCCMD_bufferTlm[i];
    }
}*/
