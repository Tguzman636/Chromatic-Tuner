/*****************************************************************************
* bsp.c for Lab2A of ECE 153a at UCSB
* Date of the Last Update:  October 27,2019
*****************************************************************************/

/**/
#include "qpn_port.h"
#include "bsp.h"
#include "lab2a.h"
#include "xintc.h"
#include "xil_exception.h"
#include <stdbool.h>



#include "xtmrctr.h"		// Timer Drivers
#include "xtmrctr_l.h" 		// Low-level timer drivers
#include "xil_printf.h" 	// Used for xil_printf()
#include <stdlib.h>
#include "lcd.h"
#include "xspi.h"
#include "xspi_l.h"

#include <stdio.h>
#include "xil_cache.h"
#include <mb_interface.h>

#include "xparameters.h"
#include <xil_types.h>
#include <xil_assert.h>

#include <xio.h>
#include "xtmrctr.h"
#include "fft.h"
#include "note.h"
#include "stream_grabber.h"
#include "trig.h"
#include "lcd.h"


#include "xintc.h"
#include "xil_exception.h"
#include "xtmrctr.h"		// Timer Drivers
#include "xtmrctr_l.h" 		// Low-level timer drivers
#include "xil_printf.h" 	// Used for xil_printf()
#include <stdlib.h>
#include "lcd.h"
#include "xspi.h"
#include "xspi_l.h"
#include "xgpio.h"


/*****************************/

/* Define all variables and Gpio objects here  */

XIntc sys_intc;
XGpio sys_enc;
XGpio sys_btn;
u32 a;
int state = 0;
int btn;
int aValue;
extern octave;

XIntc sys_intc_tmr;
XTmrCtr sys_tmrctr;
Xuint32 data;
unsigned int time = 0;
unsigned int counts = 0;
unsigned int ledNum = 1;

XGpio led;

int delayed = 0;
int ledOn = 1;

int bgNum;


void ledToggle() {
	if (ledOn) {
		ledOn = 0;
		XGpio_DiscreteWrite(&led, 1, 0);
	} else {
		ledOn = 1;
		XGpio_DiscreteWrite(&led, 1, ledNum);
	}
}

void ledRight() {
	if (ledOn) {
		if (ledNum == 1) {
			ledNum = 65536;
		}
		if (ledNum > 0) {
			ledNum = ledNum >> 1;
		}
		XGpio_DiscreteWrite(&led, 1, ledNum);
	}
}

void ledLeft() {
	if (ledOn) {
		if (ledNum == 0b1000000000000000) {
				ledNum = 1;
		} else if (ledNum < 65536) {
			ledNum = ledNum << 1;
		}
		XGpio_DiscreteWrite(&led, 1, ledNum);
	}
}

#define GPIO_CHANNEL1 1

/*..........................................................................*/
void BSP_init(void) {

	XGpio_Initialize(&led, XPAR_AXI_GPIO_LED_DEVICE_ID);

	XStatus Status;
	Status = XST_SUCCESS;
	Status = XIntc_Initialize(&sys_intc, XPAR_MICROBLAZE_0_AXI_INTC_DEVICE_ID);
	Status = XIntc_Connect(&sys_intc,XPAR_MICROBLAZE_0_AXI_INTC_ENCODER_IP2INTC_IRPT_INTR, (XInterruptHandler)TwistHandler, &sys_enc);
	Status = XIntc_Start(&sys_intc, XIN_REAL_MODE);
	XIntc_Enable(&sys_intc, XPAR_MICROBLAZE_0_AXI_INTC_ENCODER_IP2INTC_IRPT_INTR);
	Status = XGpio_Initialize(&sys_enc, XPAR_ENCODER_DEVICE_ID);
	XGpio_InterruptEnable(&sys_enc, 1);
	XGpio_InterruptGlobalEnable(&sys_enc);
	microblaze_register_handler((XInterruptHandler)XIntc_DeviceInterruptHandler, (void*)XPAR_MICROBLAZE_0_AXI_INTC_DEVICE_ID);
	microblaze_enable_interrupts();
	Status = XIntc_Connect(&sys_intc,XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_BTN_IP2INTC_IRPT_INTR, (XInterruptHandler)GpioHandler, &sys_btn);
	Status = XIntc_Start(&sys_intc, XIN_REAL_MODE);
	XIntc_Enable(&sys_intc, XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_BTN_IP2INTC_IRPT_INTR);
	Status = XGpio_Initialize(&sys_btn, XPAR_AXI_GPIO_BTN_DEVICE_ID);
	XGpio_InterruptEnable(&sys_btn, 1);
	XGpio_InterruptGlobalEnable(&sys_btn);
	Status = XIntc_Connect(&sys_intc,XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMER_0_INTERRUPT_INTR, (XInterruptHandler)extra_handler, &sys_tmrctr);
	Status = XIntc_Start(&sys_intc, XIN_REAL_MODE);
	XIntc_Enable(&sys_intc, XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMER_0_INTERRUPT_INTR);
	Status = XTmrCtr_Initialize(&sys_tmrctr, XPAR_MICROBLAZE_0_AXI_INTC_DEVICE_ID);
	XTmrCtr_SetOptions(&sys_tmrctr, 0, XTC_INT_MODE_OPTION | XTC_AUTO_RELOAD_OPTION);
	XTmrCtr_SetResetValue(&sys_tmrctr, 0, 0xFFFFFFFF - 100000);

	static XGpio dc;
	static XSpi spi;

	XSpi_Config *spiConfig;

	u32 status;
	u32 controlReg;

	status = XGpio_Initialize(&dc, XPAR_SPI_DC_DEVICE_ID);
	XGpio_SetDataDirection(&dc, 1, 0x0);
	spiConfig = XSpi_LookupConfig(XPAR_SPI_DEVICE_ID);
	status = XSpi_CfgInitialize(&spi, spiConfig, spiConfig->BaseAddress);
	XSpi_Reset(&spi);
	controlReg = XSpi_GetControlReg(&spi);
	XSpi_SetControlReg(&spi,
			(controlReg | XSP_CR_ENABLE_MASK | XSP_CR_MASTER_MODE_MASK) &
			(~XSP_CR_TRANS_INHIBIT_MASK));
	XSpi_SetSlaveSelectReg(&spi, ~0x01);

	initLCD();

	clrScr();

}
/*************************************************************************************************/
void QF_onStartup(void) {
	xil_printf("\n\rQF_onStartup\n");
	InitLUT();
	stream_grabber_start();
	setOctave(0);
	aValue = 440;
	xil_printf("Ready");
	canHistogram = 0;
}

/*************************************************************************************************/
#define SAMPLES 512 // AXI4 Streaming Data FIFO has size 512
#define M 7 //2^m=samples
#define CLOCK 100000000.0 //clock speed
#define DEC 4

int int_buffer[SAMPLES/DEC];
static float q[SAMPLES/DEC];
static float w[SAMPLES/DEC];
/**************************************************************************************************/

extern bool FFTActive;
extern bool Auto;
int int_buffer2[128];
static float q2[128];
static float w2[128];

int int_bufferMod1[16];
static float qMod1[16];
static float wMod1[16];

int int_bufferMod2[32];
static float qMod2[32];
static float wMod2[32];

int int_bufferMod3[64];
static float qMod3[64];
static float wMod3[64];

int int_bufferMod4[128];
static float qMod4[128];
static float wMod4[128];

int int_bufferMod5[128];
static float qMod5[128];
static float wMod5[128];

int int_bufferMod6[128];
static float qMod6[128];
static float wMod6[128];

int int_bufferMod7[128];
static float qMod7[128];
static float wMod7[128];

int int_bufferMod8[128];
static float qMod8[128];
static float wMod8[128];

int l;
int ticks; //used for timer
uint32_t Control;
float frequency;
float frequency2;
int extfreq;
float tot_time; //time to run program

float sample_f2;

int decimate = 32;
int mVal = 7;
int sampleVal = 4096;
void read_fsl_values(float* q, float* q2, int n, int n2, int* int_buffer2, int decimate) {
   int i;
   int j;
   unsigned int x;
   unsigned int x2;
   stream_grabber_wait_enough_samples(512);

   int sum = 0;
   int sum2 = 0;

   for (i = 0; i < n; i++) {
	   sum = sum + stream_grabber_read_sample(i);
	   if ((i + 1) % DEC == 0) {
		   int_buffer[i/DEC] = sum/DEC;
		   sum = 0;

		   x = int_buffer[i/DEC];
		   q[i/DEC] = 3.3*x/67108864.0;
	   }
   }

   for (j = 0; j < n2; j++) {
	   sum2 = sum2 + stream_grabber_read_sample(j);
	   if ((j + 1) % decimate == 0) {
		   int_buffer2[j/decimate] = sum2/decimate;
		   sum2 = 0;

		   x2 = int_buffer2[j/decimate];
		   q2[j/decimate] = 3.3*x2/67108864.0;
	   }
   }
}

void QF_onIdle(void) {

    QF_INT_UNLOCK();

    {
    	if (FFTActive && Auto) {
    	sample_f2 = 100*1000*1000/2048.0;
 	   if (octaveSelect == 1) {
 		   sampleVal = 4096;
 		   mVal = 4;
 		   decimate = 256;
 		   read_fsl_values(q,qMod1, SAMPLES, sampleVal, int_bufferMod1,decimate);
 		   stream_grabber_start();
 		   sample_f2 /= decimate;
 		   for(l=0;l<sampleVal/decimate;l++) {wMod1[l]=0;}
 		   frequency2=fft(qMod1,wMod1,sampleVal/decimate,mVal,sample_f2);
 	   } else if (octaveSelect == 2) {
 		   sampleVal = 4096;
 		   mVal = 5;
 		   decimate = 128;
 		   read_fsl_values(q,qMod2, SAMPLES, sampleVal, int_bufferMod2,decimate);
 		   stream_grabber_start();
 		   sample_f2 /= decimate;
 		   for(l=0;l<sampleVal/decimate;l++) {wMod2[l]=0;}
 		   frequency2=fft(qMod2,wMod2,sampleVal/decimate,mVal,sample_f2);
 	   } else if (octaveSelect == 3) {
 		   sampleVal = 4096;
 		   mVal = 6;
 		   decimate = 64;
 		   read_fsl_values(q,qMod3, SAMPLES, sampleVal, int_bufferMod3,decimate);
 		   stream_grabber_start();
 		   sample_f2 /= decimate;
 		   for(l=0;l<sampleVal/decimate;l++) {wMod3[l]=0;}
 		   frequency2=fft(qMod3,wMod3,sampleVal/decimate,mVal,sample_f2);
 	   } else if (octaveSelect == 4) {
 		   sampleVal = 4096;
 		   mVal = 7;
 		   decimate = 32;
 		   read_fsl_values(q,qMod4, SAMPLES, sampleVal, int_bufferMod4,decimate);
 		   stream_grabber_start();
 		   sample_f2 /= decimate;
 		   for(l=0;l<sampleVal/decimate;l++) {wMod4[l]=0;}
 		   frequency2=fft(qMod4,wMod4,sampleVal/decimate,mVal,sample_f2);
 	   } else if (octaveSelect == 5) {
 		   sampleVal = 2048;
 		   mVal = 7;
 		   decimate = 16;
 		   read_fsl_values(q,qMod5, SAMPLES, sampleVal, int_bufferMod5,decimate);
 		   stream_grabber_start();
 		   sample_f2 /= decimate;
 		   for(l=0;l<sampleVal/decimate;l++) {wMod5[l]=0;}
 		   frequency2=fft(qMod5,wMod5,sampleVal/decimate,mVal,sample_f2);
 	   } else if (octaveSelect == 6) {
 		   sampleVal = 1024;
 		   mVal = 7;
 		   decimate = 8;
 		   read_fsl_values(q,qMod6, SAMPLES, sampleVal, int_bufferMod6,decimate);
 		   stream_grabber_start();
 		   sample_f2 /= decimate;
 		   for(l=0;l<sampleVal/decimate;l++) {wMod6[l]=0;}
 		   frequency2=fft(qMod6,wMod6,sampleVal/decimate,mVal,sample_f2);
 	   } else if (octaveSelect == 7) {
 		   sampleVal = 512;
 		   mVal = 7;
 		   decimate = 4;
 		   read_fsl_values(q,qMod7, SAMPLES, sampleVal, int_bufferMod7,decimate);
 		   stream_grabber_start();
 		   sample_f2 /= decimate;
 		   for(l=0;l<sampleVal/decimate;l++) {wMod7[l]=0;}
 		   frequency2=fft(qMod7,wMod7,sampleVal/decimate,mVal,sample_f2);
 	   } else if (octaveSelect == 8) {
 		   sampleVal = 4096;
 		   mVal = 7;
 		   decimate = 2;
 		   read_fsl_values(q,qMod8, SAMPLES, sampleVal, int_bufferMod8,decimate);
 		   stream_grabber_start();
 		   sample_f2 /= decimate;
 		   for(l=0;l<sampleVal/decimate;l++) {wMod8[l]=0;}
 		   frequency2=fft(qMod8,wMod8,sampleVal/decimate,mVal,sample_f2);
 	   } else {
 		   sampleVal = 4096;
		   mVal = 7;
 		   decimate = 32;
 		   read_fsl_values(q,q2, SAMPLES, sampleVal, int_buffer2,decimate);
 		   stream_grabber_start();
 		   sample_f2 /= decimate;
 		   for(l=0;l<sampleVal/decimate;l++) {w2[l]=0;}
 		   frequency2=fft(q2,w2,sampleVal/decimate,mVal,sample_f2);
 	   }

		float cutoff = sample_f2/2;

		if(frequency < (cutoff)) {
			frequency = frequency2;
		}

		extfreq = (int)frequency2;
		findNote(frequency);
    	}
    	QActive_postISR((QActive *)&AO_Lab2A, IDLE);

    if (FFTActive && (Auto == false)) {
        	sample_f2 = 100*1000*1000/2048.0;
     	   if (octave == 1) {
     		   sampleVal = 4096;
     		   mVal = 4;
     		   decimate = 256;
     		   read_fsl_values(q,qMod1, SAMPLES, sampleVal, int_bufferMod1,decimate);
     		   stream_grabber_start();
     		   sample_f2 /= decimate;
     		   for(l=0;l<sampleVal/decimate;l++) {wMod1[l]=0;}
     		   frequency2=fft(qMod1,wMod1,sampleVal/decimate,mVal,sample_f2);
     	   } else if (octave == 2) {
     		   sampleVal = 4096;
     		   mVal = 5;
     		   decimate = 128;
     		   read_fsl_values(q,qMod2, SAMPLES, sampleVal, int_bufferMod2,decimate);
     		   stream_grabber_start();
     		   sample_f2 /= decimate;
     		   for(l=0;l<sampleVal/decimate;l++) {wMod2[l]=0;}
     		   frequency2=fft(qMod2,wMod2,sampleVal/decimate,mVal,sample_f2);
     	   } else if (octave == 3) {
     		   sampleVal = 4096;
     		   mVal = 6;
     		   decimate = 64;
     		   read_fsl_values(q,qMod3, SAMPLES, sampleVal, int_bufferMod3,decimate);
     		   stream_grabber_start();
     		   sample_f2 /= decimate;
     		   for(l=0;l<sampleVal/decimate;l++) {wMod3[l]=0;}
     		   frequency2=fft(qMod3,wMod3,sampleVal/decimate,mVal,sample_f2);
     	   } else if (octave == 4) {
     		   sampleVal = 4096;
     		   mVal = 7;
     		   decimate = 32;
     		   read_fsl_values(q,qMod4, SAMPLES, sampleVal, int_bufferMod4,decimate);
     		   stream_grabber_start();
     		   sample_f2 /= decimate;
     		   for(l=0;l<sampleVal/decimate;l++) {wMod4[l]=0;}
     		   frequency2=fft(qMod4,wMod4,sampleVal/decimate,mVal,sample_f2);
     	   } else if (octave == 5) {
     		   sampleVal = 2048;
     		   mVal = 7;
     		   decimate = 16;
     		   read_fsl_values(q,qMod5, SAMPLES, sampleVal, int_bufferMod5,decimate);
     		   stream_grabber_start();
     		   sample_f2 /= decimate;
     		   for(l=0;l<sampleVal/decimate;l++) {wMod5[l]=0;}
     		   frequency2=fft(qMod5,wMod5,sampleVal/decimate,mVal,sample_f2);
     	   } else if (octave == 6) {
     		   sampleVal = 1024;
     		   mVal = 7;
     		   decimate = 8;
     		   read_fsl_values(q,qMod6, SAMPLES, sampleVal, int_bufferMod6,decimate);
     		   stream_grabber_start();
     		   sample_f2 /= decimate;
     		   for(l=0;l<sampleVal/decimate;l++) {wMod6[l]=0;}
     		   frequency2=fft(qMod6,wMod6,sampleVal/decimate,mVal,sample_f2);
     	   } else if (octave == 7) {
     		   sampleVal = 512;
     		   mVal = 7;
     		   decimate = 4;
     		   read_fsl_values(q,qMod7, SAMPLES, sampleVal, int_bufferMod7,decimate);
     		   stream_grabber_start();
     		   sample_f2 /= decimate;
     		   for(l=0;l<sampleVal/decimate;l++) {wMod7[l]=0;}
     		   frequency2=fft(qMod7,wMod7,sampleVal/decimate,mVal,sample_f2);
     	   } else if (octave == 8) {
     		   sampleVal = 4096;
     		   mVal = 7;
     		   decimate = 2;
     		   read_fsl_values(q,qMod8, SAMPLES, sampleVal, int_bufferMod8,decimate);
     		   stream_grabber_start();
     		   sample_f2 /= decimate;
     		   for(l=0;l<sampleVal/decimate;l++) {wMod8[l]=0;}
     		   frequency2=fft(qMod8,wMod8,sampleVal/decimate,mVal,sample_f2);
     	   } else {
     		   sampleVal = 4096;
    		   mVal = 7;
     		   decimate = 32;
     		   read_fsl_values(q,q2, SAMPLES, sampleVal, int_buffer2,decimate);
     		   stream_grabber_start();
     		   sample_f2 /= decimate;
     		   for(l=0;l<sampleVal/decimate;l++) {w2[l]=0;}
     		   frequency2=fft(q2,w2,sampleVal/decimate,mVal,sample_f2);
     	   }

    		float cutoff = sample_f2/2;

    		if(frequency < (cutoff)) {
    			frequency = frequency2;
    		}

    		extfreq = (int)frequency2;
    		findNote(frequency);
        	}
        	QActive_postISR((QActive *)&AO_Lab2A, IDLE);
        }
}
/*..........................................................................*/
void Q_onAssert(char const Q_ROM * const Q_ROM_VAR file, int line) {
    (void)file;                                   /* avoid compiler warning */
    (void)line;                                   /* avoid compiler warning */
    QF_INT_LOCK();
    for (;;) {
    }
}

/*..........................................................................*/
void GpioHandler(void *CallbackRef) {
	time = 0;
	XTmrCtr_Start(&sys_tmrctr, 0);
	btn = XGpio_DiscreteRead(&sys_btn, 1);
		if (btn == 1) {
			QActive_postISR((QActive *)&AO_Lab2A, BUTTON_PRESS_UP);
		}
		if (btn == 2) {
			QActive_postISR((QActive *)&AO_Lab2A, BUTTON_PRESS_LEFT);
		}
		if (btn == 4) {
			QActive_postISR((QActive *)&AO_Lab2A, BUTTON_PRESS_RIGHT);
		}
		if (btn == 8) {
			QActive_postISR((QActive *)&AO_Lab2A, BUTTON_PRESS_DOWN);
		}
		if (btn == 16) {
			QActive_postISR((QActive *)&AO_Lab2A, BUTTON_PRESS_CENTER);
		}
		XGpio_InterruptClear(&sys_btn, XGPIO_IR_MASK);
}

void TwistHandler(void *CallbackRef) {
	time = 0;
	XTmrCtr_Start(&sys_tmrctr, 0);

	if (!delayed) {
		delayed = 1;
	}

	int EncStatus = XGpio_DiscreteRead(&sys_enc, 1);
	switch(state) {
	   case 0:
		   if (EncStatus == 1) {
			   state = 1;
		   } else if (EncStatus == 2) {
			   state = 4;
		   }
		  break;
	   case 1:
		  if (EncStatus == 0) {
			  state = 2;
		  } else if (EncStatus == 3) {
			  state = 0;
		  }
		  break;
	   case 2:
		   if (EncStatus == 2) {
			   state = 3;
		   }
		   break;
	   case 3:
		   if (EncStatus == 3) {
			   state = 0;
			   ledRight();
			   QActive_postISR((QActive *)&AO_Lab2A, ENCODER_UP);
		   }
		   break;
	   case 4:
		   if (EncStatus == 0) {
			   state = 5;
		   } else if (EncStatus == 3) {
			   state = 0;
		   }
		   break;
	   case 5:
		   if (EncStatus == 1) {
			   state = 6;
		   }
		   break;
	   case 6:
		   if (EncStatus == 3) {
			   state = 0;
			  ledLeft();
			  QActive_postISR((QActive *)&AO_Lab2A, ENCODER_DOWN);
		   }
	}

	if ((EncStatus == 7)) {
		QActive_postISR((QActive *)&AO_Lab2A, ENCODER_CLICK);
	}

	XGpio_InterruptClear(&sys_enc, XGPIO_IR_MASK);
}

void extra_handler(void *CallbackRef) {
	Xuint32 ControlStatusReg;
	ControlStatusReg = XTimerCtr_ReadReg(sys_tmrctr.BaseAddress, 0, XTC_TCSR_OFFSET);
	time++;

	XGpio led;
	XGpio_Initialize(&led, XPAR_AXI_GPIO_LED_DEVICE_ID);
	XTmrCtr_WriteReg(sys_tmrctr.BaseAddress, 0, XTC_TCSR_OFFSET, ControlStatusReg |XTC_CSR_INT_OCCURED_MASK);
};

