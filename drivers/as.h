// *************************************************************************************************
//
//	Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
//
//
//	  Redistribution and use in source and binary forms, with or without
//	  modification, are permitted provided that the following conditions
//	  are met:
//
//	    Redistributions of source code must retain the above copyright
//	    notice, this list of conditions and the following disclaimer.
//
//	    Redistributions in binary form must reproduce the above copyright
//	    notice, this list of conditions and the following disclaimer in the
//	    documentation and/or other materials provided with the
//	    distribution.
//
//	    Neither the name of Texas Instruments Incorporated nor the names of
//	    its contributors may be used to endorse or promote products derived
//	    from this software without specific prior written permission.
//
//	  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//	  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//	  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//	  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//	  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//	  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//	  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//	  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//	  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//	  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//	  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *************************************************************************************************

#ifndef AS_H_
#define AS_H_


// *************************************************************************************************
// Include section

// *************************************************************************************************
// Prototypes section
extern void as_init(void);
extern void as_start(void);
extern void as_stop(void);
extern uint8_t as_read_register(uint8_t bAddress);
extern uint8_t as_write_register(uint8_t bAddress, uint8_t bData);

// *************************************************************************************************
// Defines section

// Disconnect power supply for acceleration sensor when not used
#define AS_DISCONNECT

// Port and pin resource for SPI interface to acceleration sensor
// SDO=MOSI=P1.6, SDI=MISO=P1.5, SCK=P1.7
#define AS_SPI_IN            (P1IN)
#define AS_SPI_OUT           (P1OUT)
#define AS_SPI_DIR           (P1DIR)
#define AS_SPI_SEL           (P1SEL)
#define AS_SPI_REN           (P1REN)
#define AS_SDO_PIN           (BIT6)
#define AS_SDI_PIN           (BIT5)
#define AS_SCK_PIN           (BIT7)

// CSN=PJ.1
#define AS_CSN_OUT           (PJOUT)
#define AS_CSN_DIR           (PJDIR)
#define AS_CSN_PIN           (BIT1)

#define AS_TX_BUFFER         (UCA0TXBUF)
#define AS_RX_BUFFER         (UCA0RXBUF)
#define AS_TX_IFG            (UCTXIFG)
#define AS_RX_IFG            (UCRXIFG)
#define AS_IRQ_REG           (UCA0IFG)
#define AS_SPI_CTL0          (UCA0CTL0)
#define AS_SPI_CTL1          (UCA0CTL1)
#define AS_SPI_BR0           (UCA0BR0)
#define AS_SPI_BR1           (UCA0BR1)

// Port and pin resource for power-up of acceleration sensor, VDD=PJ.0
#define AS_PWR_OUT           (PJOUT)
#define AS_PWR_DIR           (PJDIR)
#define AS_PWR_PIN           (BIT0)

// Port, pin and interrupt resource for interrupt from acceleration sensor, CMA_INT=P2.5
#define AS_INT_IN            (P2IN)
#define AS_INT_OUT           (P2OUT)
#define AS_INT_DIR           (P2DIR)
#define AS_INT_IE            (P2IE)
#define AS_INT_IES           (P2IES)
#define AS_INT_IFG           (P2IFG)
#define AS_INT_PIN           (BIT5)

// SPI timeout to detect sensor failure
#define AS_SPI_TIMEOUT       (1000u)

/* register address: */
#define ADDR_CTRL	    (0x02)
#define ADDR_INT_STATUS     (0x05)
#define ADDR_MDTHR          (0x09)
#define ADDR_MDFFTMR        (0x0A)
#define ADDR_FFTHR	    (0x0B)

/* defines for sampling rate */
/* The first one should be 400 but must fit u8 so divide by 10 */
#define SAMPLING_400_HZ	    (40)
#define SAMPLING_40_HZ	    (4)
#define SAMPLING_100_HZ	    (10)
#define SAMPLING_10_HZ	    (1)

/******************************************************************************/
/* Global Variable section */

/* Set system flags */
typedef union {
	struct {
		uint8_t motiondet	: 2;    /* MDET see AS_MOTION_STATUS */
		uint8_t falldet    	: 1;    /* FFDET see AS_FALL_STATUS */
		uint8_t reserved        : 5;    /* reserved, initial value = 0h */
	} int_status;
	/* Shortcut to all display flags (for reset) */
	uint8_t all_flags;
} as_status_register_flags;
extern volatile as_status_register_flags as_status;

// *************************************************************************************************
// Global Variable section

struct As_Param {
	/* configuration bits for motion and free fall */
	uint8_t	MDTHR;
	uint8_t	MDFFTMR;
	uint8_t	FFTHR;
	uint8_t	sampling;
	uint8_t	range;
	uint8_t	mode;
};
// *************************************************************************************************
// Extern section
struct As_Param as_config;

enum AS_MOTION_STATUS {
	AS_NO_MOTION = 00,	/* motion not detected */
	AS_TRIGGER_X = 01,	/* motion trigger on x */
	AS_TRIGGER_Y = 10,	/* motion trigger on y */
	AS_TRIGGER_Z = 11	/* motion trigger on z */
};
extern enum AS_MOTION_STATUS as_motion_bits;

enum AS_FALL_STATUS {
	AS_NOFALL = 0,    /* free fall not detected */
	AS_FALL           /* free fall detected */
};
extern enum AS_FALL_STATUS as_fall_bit;


#define FALL_MODE 0
#define MEASUREMENT_MODE 1
#define ACTIVITY_MODE 2

/*  when activity mode is configured you can set this flag
    set 1 to remain in the motion detection mode
    set 0 to switch at measurment mode at 400 Hz */
#define MDET_EXIT 1

// Global flag for proper acceleration sensor operation
extern uint8_t as_ok;
volatile uint8_t as_last_interrupt;

#endif /*AS_H_*/
