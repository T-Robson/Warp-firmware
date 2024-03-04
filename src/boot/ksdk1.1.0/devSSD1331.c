#include <stdint.h>

#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t	inBuffer[32];
volatile uint8_t	payloadBytes[32];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
// enum
// {
// 	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
// 	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
// 	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
// 	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
// 	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
// };



static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */

 	// SEGGER_RTT_WriteString(0, "\r\n\tWriting Byte\n");

	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}

void
writeArray(int count, uint8_t * commandArray){

	for (int i = 0; i < count; i++)
	{
		writeCommand(commandArray[i]);
	};
	
}


int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	warpEnableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x06);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0x50);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel


	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);

	return 0;
}

int
devSSD1331demo(void){

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	
	writeArray(2, (uint8_t[]){ kSSD1331CommandFILL, 0x01});

	// writeCommand(kSSD1331CommandFILL);
	// writeCommand(0x01);

	/*
	 *	Clear Screen
	 */

	writeArray(5, (uint8_t[]){ kSSD1331CommandCLEAR, 0x00, 0x00, 0x5F, 0x3F});

    /*
     *	Read the manual for the SSD1331 (SSD1331_1.2.pdf) to figure
     *	out how to fill the entire screen with the brightest shade
     *	of green.
     */

    //Set max contrast (0xFF) on each colour channel for brightest green

	writeArray(6, (uint8_t[]){ 0x81, 0xFF, 0x82, 0xFF, 0x83, 0xFF});
    
    SEGGER_RTT_WriteString(0, "\r\n\tContrast set to maximum\n");

    // Set maximum pixel current for maximum brightness
	writeArray(2, (uint8_t[]){ 0x87, 0x0F});

    SEGGER_RTT_WriteString(0, "\r\n\tCurrent set to maximum\n");

    // Set highest first precharge level
	writeArray( 8,
	(uint8_t[]){
		0xBB, 0x3E, /* Set PrechargeLevel*/
		0x8A, 0xFF, /* Set second precharge levels*/
		0x8B, 0xFF,
		0x8C, 0xFF
	}
	);
	
    SEGGER_RTT_WriteString(0, "\r\n\tPrecharge set to maximum\n");

    // Draws green rectangle covering whole screen 
	writeArray(11, (uint8_t[]){
		0x22,				/* DrawRect command */
		0x00, 0x00,			/* Start Col, Row */
		0x5F, 0x3F,			/* End Col, Row */
		0x00, 0x3F, 0x00,	/* Outline RGB */
		0x00, 0x3F, 0x00	/* Fill RGB */
	});

    SEGGER_RTT_WriteString(0, "\r\n\tGreen rectangle drawn\n");

#if 0

	writeArray(11, (uint8_t[]){
		0x22,				/* DrawRect command */
		0x06, 0x09,			/* Start Col, Row */
		0x56, 0x26,			/* End Col, Row */
		0x00, 0x00, 0x00,	/* Outline RGB */
		0x3F, 0x00, 0x00	/* Fill RGB */
	});

	/* prints 4b25 to the screen*/
	writeArray(506,(uint8_t[]){
		0x22, 0x10, 0x0C, 0x14, 0x10, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x20, 0x0C, 0x24, 0x10, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x24, 0x0C, 0x28, 0x10, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x34, 0x0C, 0x38, 0x10, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x38, 0x0C, 0x3C, 0x10, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x48, 0x0C, 0x4C, 0x10, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x4C, 0x0C, 0x50, 0x10, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x0C, 0x10, 0x10, 0x14, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x10, 0x10, 0x14, 0x14, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x1C, 0x10, 0x20, 0x14, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x28, 0x10, 0x2C, 0x14, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x30, 0x10, 0x34, 0x14, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x3C, 0x10, 0x40, 0x14, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x44, 0x10, 0x48, 0x14, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x08, 0x14, 0x0C, 0x18, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x10, 0x14, 0x14, 0x18, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x1C, 0x14, 0x20, 0x18, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x20, 0x14, 0x24, 0x18, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x24, 0x14, 0x28, 0x18, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x3C, 0x14, 0x40, 0x18, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x48, 0x14, 0x4C, 0x18, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x4C, 0x14, 0x50, 0x18, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x08, 0x18, 0x0C, 0x1C, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x0C, 0x18, 0x10, 0x1C, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x10, 0x18, 0x14, 0x1C, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x14, 0x18, 0x18, 0x1C, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x1C, 0x18, 0x20, 0x1C, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x28, 0x18, 0x2C, 0x1C, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x38, 0x18, 0x3C, 0x1C, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x50, 0x18, 0x54, 0x1C, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x10, 0x1C, 0x14, 0x20, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x1C, 0x1C, 0x20, 0x20, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x28, 0x1C, 0x2C, 0x20, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x34, 0x1C, 0x38, 0x20, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x44, 0x1C, 0x48, 0x20, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x50, 0x1C, 0x54, 0x20, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x10, 0x20, 0x14, 0x24, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x1C, 0x20, 0x20, 0x24, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x20, 0x20, 0x24, 0x24, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x24, 0x20, 0x28, 0x24, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x30, 0x20, 0x34, 0x24, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x34, 0x20, 0x38, 0x24, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x38, 0x20, 0x3C, 0x24, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x3C, 0x20, 0x40, 0x24, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x48, 0x20, 0x4C, 0x24, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F,
		0x22, 0x4C, 0x20, 0x50, 0x24, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F
	});



	writeArray(7, (uint8_t[]){
		0x27, /* Scroll Vertically*/
		0x02,
		0x00,
		0x40,
		0x01,
		0x00,
		0x2F
	}	
	);

#endif


	return 0;
}