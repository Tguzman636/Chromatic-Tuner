///*
// * Copyright (c) 2009-2012 Xilinx, Inc.  All rights reserved.
// *
// * Xilinx, Inc.
// * XILINX IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS" AS A
// * COURTESY TO YOU.  BY PROVIDING THIS DESIGN, CODE, OR INFORMATION AS
// * ONE POSSIBLE   IMPLEMENTATION OF THIS FEATURE, APPLICATION OR
// * STANDARD, XILINX IS MAKING NO REPRESENTATION THAT THIS IMPLEMENTATION
// * IS FREE FROM ANY CLAIMS OF INFRINGEMENT, AND YOU ARE RESPONSIBLE
// * FOR OBTAINING ANY RIGHTS YOU MAY REQUIRE FOR YOUR IMPLEMENTATION.
// * XILINX EXPRESSLY DISCLAIMS ANY WARRANTY WHATSOEVER WITH RESPECT TO
// * THE ADEQUACY OF THE IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO
// * ANY WARRANTIES OR REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE
// * FROM CLAIMS OF INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
// * AND FITNESS FOR A PARTICULAR PURPOSE.
// *
// */
//
///*
// * helloworld.c: simple test application
// *
// * This application configures UART 16550 to baud rate 9600.
// * PS7 UART (Zynq) is not initialized by this application, since
// * bootrom/bsp configures it to baud rate 115200
// *
// * ------------------------------------------------
// * | UART TYPE   BAUD RATE                        |
// * ------------------------------------------------
// *   uartns550   9600
// *   uartlite    Configurable only in HW design
// *   ps7_uart    115200 (configured by bootrom/bsp)
// */
//
//#include <stdio.h>
//#include "xil_cache.h"
//#include <mb_interface.h>
//
//#include "xparameters.h"
//#include <xil_types.h>
//#include <xil_assert.h>
//
//#include <xio.h>
//#include "xtmrctr.h"
//#include "fft.h"
//#include "note.h"
//#include "stream_grabber.h"
//#include "trig.h"
//#include "lcd.h"
//
//
//#include "xintc.h"
//#include "xil_exception.h"
//#include "xtmrctr.h"		// Timer Drivers
//#include "xtmrctr_l.h" 		// Low-level timer drivers
//#include "xil_printf.h" 	// Used for xil_printf()
//#include <stdlib.h>
//#include "lcd.h"
//#include "xspi.h"
//#include "xspi_l.h"
//#include "xgpio.h"
//
//
///*************************************************************************************************/
//#define SAMPLES 512 // AXI4 Streaming Data FIFO has size 512
//#define M 7 //2^m=samples
//#define CLOCK 100000000.0 //clock speed
//#define DEC 4
//
//int int_buffer[SAMPLES/DEC];
//static float q[SAMPLES/DEC];
//static float w[SAMPLES/DEC];
///**************************************************************************************************/
//
//
//
//
//#define SAMPLES2 4096 // AXI4 Streaming Data FIFO has size 512
//#define M2 7 //2^m=samples
//#define DEC2 32
//
//int int_buffer2[SAMPLES2/DEC2];
//static float q2[SAMPLES2/DEC2];
//static float w2[SAMPLES2/DEC2];
//
//
//
//
//
//
//
//void read_fsl_values(float* q, float* q2, int n, int n2) {
//   int i;
//   int j;
//   unsigned int x;
//
//   unsigned int x2;
//   //stream_grabber_start();
//   stream_grabber_wait_enough_samples(SAMPLES);
//
//   int sum = 0;
//
//   int sum2 = 0;
//
//   for (i = 0; i < n; i++) {
//	   sum = sum + stream_grabber_read_sample(i);
//	   if ((i + 1) % DEC == 0) {
//		   int_buffer[i/DEC] = sum/DEC;
//		   sum = 0;
//
//		   x = int_buffer[i/DEC];
//		   q[i/DEC] = 3.3*x/67108864.0;
//	   }
//   }
//
//   for (j = 0; j < n2; j++) {
//	   sum2 = sum2 + stream_grabber_read_sample(j);
//	   if ((j + 1) % DEC2 == 0) {
//		   int_buffer2[j/DEC2] = sum2/DEC2;
//		   sum2 = 0;
//
//		   x2 = int_buffer2[j/DEC2];
//		   q2[j/DEC2] = 3.3*x2/67108864.0;
//	   }
//   }
//}
//
//int main() {
//   float sample_f;
//   int l;
//   int ticks; //used for timer
//   uint32_t Control;
//   float frequency;
//   float tot_time; //time to run program
//
//
//   float frequency2;
//   float sample_f2;
//
//
//   Xil_ICacheInvalidate();
//   Xil_ICacheEnable();
//   Xil_DCacheInvalidate();
//   Xil_DCacheEnable();
//
//   //set up timer
//   XTmrCtr timer;
//   XTmrCtr_Initialize(&timer, XPAR_AXI_TIMER_0_DEVICE_ID);
//   Control = XTmrCtr_GetOptions(&timer, 0) | XTC_CAPTURE_MODE_OPTION | XTC_INT_MODE_OPTION;
//   XTmrCtr_SetOptions(&timer, 0, Control);
//
//
//   print("Hello World\n\r");
//
//
//
//
//
////   static XGpio dc;
////   	static XSpi spi;
////
////   	XSpi_Config *spiConfig;	/* Pointer to Configuration data */
////
////   	u32 status;
////   	u32 controlReg;
////   /*
////   	 * Initialize the GPIO driver so that it's ready to use,
////   	 * specify the device ID that is generated in xparameters.h
////   	 */
////   	status = XGpio_Initialize(&dc, XPAR_SPI_DC_DEVICE_ID);
////   	if (status != XST_SUCCESS)  {
////   		xil_printf("Initialize GPIO dc fail!\n");
////   		return XST_FAILURE;
////   	}
////
////   	/*
////   	 * Set the direction for all signals to be outputs
////   	 */
////   	XGpio_SetDataDirection(&dc, 1, 0x0);
////
////
////
////   	/*
////   	 * Initialize the SPI driver so that it is  ready to use.
////   	 */
////   	spiConfig = XSpi_LookupConfig(XPAR_SPI_DEVICE_ID);
////   	if (spiConfig == NULL) {
////   		xil_printf("Can't find spi device!\n");
////   		return XST_DEVICE_NOT_FOUND;
////   	}
////
////   	status = XSpi_CfgInitialize(&spi, spiConfig, spiConfig->BaseAddress);
////   	if (status != XST_SUCCESS) {
////   		xil_printf("Initialize spi fail!\n");
////   		return XST_FAILURE;
////   	}
////
////   	/*
////   	 * Reset the SPI device to leave it in a known good state.
////   	 */
////   	XSpi_Reset(&spi);
////
////   	/*
////   	 * Setup the control register to enable master mode
////   	 */
////   	controlReg = XSpi_GetControlReg(&spi);
////   	XSpi_SetControlReg(&spi,
////   			(controlReg | XSP_CR_ENABLE_MASK | XSP_CR_MASTER_MODE_MASK) &
////   			(~XSP_CR_TRANS_INHIBIT_MASK));
////
////   	// Select 1st slave device
////   	XSpi_SetSlaveSelectReg(&spi, ~0x01);
////   initLCD();
////
////   	clrScr();
////   	drawBg(1);
//
//
//
//
//
//
//
//
//
//
//
//   InitLUT();
//
//   stream_grabber_start();
//   while(1) {
//      XTmrCtr_Start(&timer, 0);
//
//      //Read Values from Microblaze buffer, which is continuously populated by AXI4 Streaming Data FIFO.
//
//
//      read_fsl_values(q,q2, SAMPLES, SAMPLES2);
//
//
//
//      stream_grabber_start();
//
//
//      sample_f = 100*1000*1000/2048.0;
//      sample_f /=DEC;
//
//      sample_f2 = 100*1000*1000/2048.0;
//      sample_f2 /=DEC2;
//      //xil_printf("sample frequency: %d \r\n",(int)sample_f);
//
//      //zero w array
//      for(l=0;l<SAMPLES/DEC;l++)
//         w[l]=0;
//
//
//
//      for(l=0;l<SAMPLES2/DEC2;l++)
//    	  w2[l]=0;
//
//
//
//
//      frequency=fft(q,w,SAMPLES/DEC,M,sample_f);
//
//
//
//
//      frequency2=fft(q2,w2,SAMPLES2/DEC2,M2,sample_f2);
//
//
//
//      findNote(frequency);
//
//      //get time to run program
//      ticks=XTmrCtr_GetValue(&timer, 0);
//      XTmrCtr_Stop(&timer, 0);
//      tot_time=ticks/CLOCK;
//     xil_printf("program time: %dms \r\n",(int)(1000*tot_time));
//
//      //ignore noise below set frequency
//      if(frequency < 750.0) {
//         xil_printf("frequency: %d Hz\r\n", (int)(frequency2+.5));
//      } else {
//    	  xil_printf("frequency: %d Hz\r\n", (int)(frequency+.5));
//      }
//
//   }
//
//
//   return 0;
//}
/*****************************************************************************
* main.c for Lab2A of ECE 153a at UCSB
* Date of the Last Update:  November 1,2020
*****************************************************************************/

#include "qpn_port.h"                                       /* QP-nano port */
#include "bsp.h"                             /* Board Support Package (BSP) */
#include "lab2a.h"                               /* application interface */
#include "xil_cache.h"		                /* Cache Drivers */


static QEvent l_lab2aQueue[30];

QActiveCB const Q_ROM Q_ROM_VAR QF_active[] = {
	{ (QActive *)0,            (QEvent *)0,          0                    },
	{ (QActive *)&AO_Lab2A,    l_lab2aQueue,         Q_DIM(l_lab2aQueue)  }
};

Q_ASSERT_COMPILE(QF_MAX_ACTIVE == Q_DIM(QF_active) - 1);

// Do not edit main, unless you have a really good reason
int main(void) {

	Xil_ICacheInvalidate();
	Xil_ICacheEnable();
	Xil_DCacheInvalidate();
	Xil_DCacheEnable();

	Lab2A_ctor(); // inside of lab2a.c
	BSP_init(); // inside of bsp.c, starts out empty!
	QF_run(); // inside of qfn.c
	return 0;
}
