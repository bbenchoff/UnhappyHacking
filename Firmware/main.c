/* 
##############################
#	
#	Unhappy Hacking Keyboard
#	     ~ the code ~
#	
#	
############################## 
*/

#include "usbdrv/usbdrv.h"
#include "usbdrv/oddebug.h"
#include "usbconfig.h"


#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>

//Space Button
// keycode 44
#define BUTTON_PORT_B1 PORTB       /* PORTx - register for BUTTON 1 output */
#define BUTTON_PIN_B1 PINB         /* PINx - register for BUTTON 1 input */
#define BUTTON_BIT_B1 PB1          /* bit for BUTTON 1 input/output */

//Enter Button
// keycode 40
#define BUTTON_PORT_B3 PORTB       /* PORTx - register for BUTTON 3 output */
#define BUTTON_PIN_B3 PINB         /* PINx - register for BUTTON 3 input */
#define BUTTON_BIT_B3 PB4          /* bit for BUTTON 3 input/output */
#define BUTTON_BIT_B3 PB4          /* bit for BUTTON 3 input/output */

// '1' Button
// keycode 30
#define BUTTON_PORT_B2 PORTB       /* PORTx - register for BUTTON 2 output */
#define BUTTON_PIN_B2 PINB         /* PINx - register for BUTTON 2 input */
#define BUTTON_BIT_B2 PB3          /* bit for BUTTON 2 input/output */

// '0' Button
// keycode 39
#define BUTTON_PORT_B4 PORTB       /* PORTx - register for BUTTON 4 output */
#define BUTTON_PIN_B4 PINB         /* PINx - register for BUTTON 4 input */
#define BUTTON_BIT_B4 PB5          /* bit for BUTTON 4 input/output */


/* ------------------------------------------------------------------------- */

static uchar    reportBuffer[8] = {0,0,0,0,0,0,0,0};    /* buffer for HID reports */

/* Reportbuffer format:

	0  Modifier byte
	1  reserved
	2  keycode array (0)
	3  keycode array (1)
	4  keycode array (2)
	5  keycode array (3)
	6  keycode array (4)
	7  keycode array (5)
	
	<< This is the standard usb-keyboard reportbuffer. It allows for 6 simultaneous keypresses to be detected (excl. modifier keys). In this application we only use 1, so the last 5 bytes in this buffer will always remain 0. >>
	<< I decided not to optimize this in order to make it easy to add extra keys that can be pressed simultaneously>>
	
   Modifier byte: 8 bits, each individual bit represents one of the modifier keys.

   	bit0  LEFT CTRL		(1<<0)
	bit1  LEFT SHIFT	(1<<1)
	bit2  LEFT ALT		(1<<2)
	bit3  LEFT GUI		(1<<3)
	bit4  RIGHT CTRL	(1<<4)
	bit5  RIGHT SHIFT	(1<<5)
	bit6  RIGHT ALT		(1<<6)
	bit7  RIGHT GUI		(1<<7)

	an example of a reportBuffer for a CTRL+ALT+Delete keypress:

	{((1<<0)+(1<<2)),0,76,0,0,0,0,0}

	the first byte holds both the LEFT CTRL and LEFT  modifier keys the 3rd byte holds the delete key (== decimal 76)

*/

static uchar    idleRate;           /* in 4 ms units */
static uchar    newReport = 0;		/* current report */

static uchar    buttonState_B1 = 3;		/*  stores state of button 0 */
static uchar    buttonState_B2 = 3;		/*  stores state of button 1 */
static uchar    buttonState_B3 = 3;		/*  stores state of button 2 */
static uchar    buttonState_B4 = 3;		/*  stores state of button 3 */

static uchar    buttonChanged_B1;		
static uchar    buttonChanged_B2;		
static uchar    buttonChanged_B3;		
static uchar    buttonChanged_B4;		

static uchar	debounceTimeIsOver = 1;	/* for switch debouncing */


/* ------------------------------------------------------------------------- */

const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
 0x05, 0x01, // USAGE_PAGE (Generic Desktop)
 0x09, 0x06, // USAGE (Keyboard)
 0xa1, 0x01, // COLLECTION (Application)
 0x05, 0x07, // USAGE_PAGE (Keyboard)
 0x19, 0xe0, // USAGE_MINIMUM (Keyboard LeftControl)
 0x29, 0xe7, // USAGE_MAXIMUM (Keyboard Right GUI)
 0x15, 0x00, // LOGICAL_MINIMUM (0)
 0x25, 0x01, // LOGICAL_MAXIMUM (1)
 0x75, 0x01, // REPORT_SIZE (1)
 0x95, 0x08, // REPORT_COUNT (8)
 0x81, 0x02, // INPUT (Data,Var,Abs)
 0x95, 0x01, // REPORT_COUNT (1)
 0x75, 0x08, // REPORT_SIZE (8)
 0x81, 0x03, // INPUT (Cnst,Var,Abs)
 0x95, 0x05, // REPORT_COUNT (5)
 0x75, 0x01, // REPORT_SIZE (1)
 0x05, 0x08, // USAGE_PAGE (LEDs)
 0x19, 0x01, // USAGE_MINIMUM (Num Lock)
 0x29, 0x05, // USAGE_MAXIMUM (Kana)
 0x91, 0x02, // OUTPUT (Data,Var,Abs)
 0x95, 0x01, // REPORT_COUNT (1)
 0x75, 0x03, // REPORT_SIZE (3)
 0x91, 0x03, // OUTPUT (Cnst,Var,Abs)
 0x95, 0x06, // REPORT_COUNT (6)
 0x75, 0x08, // REPORT_SIZE (8)
 0x15, 0x00, // LOGICAL_MINIMUM (0)
 0x25, 0x65, // LOGICAL_MAXIMUM (101)
 0x05, 0x07, // USAGE_PAGE (Keyboard)
 0x19, 0x00, // USAGE_MINIMUM (Reserved (no event indicated))
 0x29, 0x65, // USAGE_MAXIMUM (Keyboard Application)
 0x81, 0x00, // INPUT (Data,Ary,Abs)
 0xc0 // END_COLLECTION
};

 
static void timerPoll(void)
{
	static unsigned int timerCnt;

    if(TIFR & (1 << TOV1)){
        TIFR = (1 << TOV1); /* clear overflow */
        if(++timerCnt >= 5){       // 5/63 sec delay for switch debouncing
			timerCnt = 0;
			debounceTimeIsOver = 1; 
        }
    }
}

static void buildReport(void){
	
	uchar key; 

	if(newReport == 0){	
		if (buttonChanged_B1 == 1){
        	if (buttonState_B1 != 0){ // if button 1 is released
				key = 0; //button released event
			} 
			else { //if button 1 is pressed
				key = 44; // key = space
	    	}
			buttonChanged_B1 = 0;
			reportBuffer[2] = key;
		}

		if (buttonChanged_B2 == 1){
        	if (buttonState_B2 != 0){ // if button 2 is pressed
				key = 0; //button released event
			} 
			else {
				key = 40;  // key = enter
			}
			buttonChanged_B2 = 0;
    		reportBuffer[3] = key;
    	}
		if(buttonChanged_B3 == 1){
        	if (buttonState_B3 != 0){ // if button 3 is pressed
				key = 0; //button released event
			} 
			else {
				key = 30; // key = '1'
			}
			buttonChanged_B3 = 0;
			reportBuffer[4] = key;
	    }
		if(buttonChanged_B4 == 1){
        	if (buttonState_B4 != 0){ // if button 4 is pressed
				key = 0; //button released event
			} 
			else {
				key = 39;  // key = '0'
    		}
			buttonChanged_B4 = 0;
    		reportBuffer[5] = key;
    	}
	
		newReport = 1; //if no button has changed, the previous report will be sent
	}
}

static void checkButtonChange(void) {
	
	uchar tempButtonValue_B1 = bit_is_set(BUTTON_PIN_B1, BUTTON_BIT_B1); //status of switch is stored in tempButtonValue 
	uchar tempButtonValue_B2 = bit_is_set(BUTTON_PIN_B2, BUTTON_BIT_B2); //status of switch is stored in tempButtonValue 
	uchar tempButtonValue_B3 = bit_is_set(BUTTON_PIN_B3, BUTTON_BIT_B3);  //status of switch is stored in tempButtonValue 
	uchar tempButtonValue_B4 = bit_is_set(BUTTON_PIN_B4, BUTTON_BIT_B4);  //status of switch is stored in tempButtonValue 

	if (tempButtonValue_B1 != buttonState_B1){ //if status has changed
		buttonState_B1 = tempButtonValue_B1;	// change buttonState to new state
		debounceTimeIsOver = 0;	// debounce timer starts
		newReport = 0; // initiate new report 
		buttonChanged_B1 = 1;
	}
	if (tempButtonValue_B2 != buttonState_B2){ //if status has changed
		buttonState_B2 = tempButtonValue_B2;	// change buttonState to new state
		debounceTimeIsOver = 0;	// debounce timer starts
		newReport = 0; // initiate new report 
		buttonChanged_B2 = 1;
	}
	if (tempButtonValue_B3 != buttonState_B3){ //if status has changed
		buttonState_B3 = tempButtonValue_B3;	// change buttonState to new state
		debounceTimeIsOver = 0;	// debounce timer starts
		newReport = 0; // initiate new report 
		buttonChanged_B3 = 1;
	}
	if (tempButtonValue_B4 != buttonState_B4){ //if status has changed
		buttonState_B4 = tempButtonValue_B4;	// change buttonState to new state
		debounceTimeIsOver = 0;	// debounce timer starts
		newReport = 0; // initiate new report 
		buttonChanged_B4 = 1;
	}

}

/* ------------------------------------------------------------------------- */

static void timerInit(void)
{
    TCCR1 = 0x0b;           /* select clock: 16.5M/1k -> overflow rate = 16.5M/256k = 62.94 Hz */
}


/* -------------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* -------------------------------------------------------------------------------- */

uchar	usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    usbMsgPtr = reportBuffer;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            buildReport();
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
	return 0;
}


/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void)
{
uchar       step = 128;
uchar       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

void    hadUsbReset(void)
{
    calibrateOscillator();
    eeprom_write_byte(0, OSCCAL);   /* store the calibrated value in EEPROM byte 0*/
}


/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

int main(void)
{
	uchar   i;
	uchar   calibrationValue;

	do {} while (!eeprom_is_ready());
    calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }
    
    
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(100);
    }
    usbDeviceConnect();
	
	usbInit();

    wdt_enable(WDTO_2S);

	/* turn on internal pull-up resistor for the switches */
    BUTTON_PORT_B1 |= _BV(BUTTON_BIT_B1);
	BUTTON_PORT_B2 |= _BV(BUTTON_BIT_B2);
	BUTTON_PORT_B3 |= _BV(BUTTON_BIT_B3);
	BUTTON_PORT_B4 |= _BV(BUTTON_BIT_B4);

    timerInit();
	

    sei();

    for(;;){    /* main event loop */
        wdt_reset();
        usbPoll();
		if (debounceTimeIsOver == 1){
			checkButtonChange();
		}

		if(usbInterruptIsReady() && newReport == 0){ /* we can send another report */
        	buildReport();
           	usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
        	}
        
		timerPoll();
	}
   	return 0;
}
