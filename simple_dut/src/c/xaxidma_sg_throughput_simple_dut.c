/******************************************************************************
* Copyright (C) 2010 - 2022 Xilinx, Inc.  All rights reserved.
* Copyright (C) 2022 - 2023 Advanced Micro Devices, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/

/*****************************************************************************/
/**
 *
 * @file xaxidma_example_sg_intr.c
 *
 * This file demonstrates how to use the xaxidma driver on the Xilinx AXI
 * DMA core (AXIDMA) to transfer packets in interrupt mode when the AXIDMA
 * core is configured in Scatter Gather Mode
 *
 * We show how to do multiple packets transfers, as well as how to do multiple
 * BDs per packet transfers.
 *
 * This code assumes a loopback hardware widget is connected to the AXI DMA
 * core for data packet loopback.
 *
 * To see the debug print, you need a Uart16550 or uartlite in your system,
 * and please set "-DDEBUG" in your compiler options. You need to rebuild your
 * software executable.
 *
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who  Date     Changes
 * ----- ---- -------- -------------------------------------------------------
 * 1.00a jz   05/18/10 First release
 * 2.00a jz   08/10/10 Second release, added in xaxidma_g.c, xaxidma_sinit.c,
 *		       		   updated tcl file, added xaxidma_porting_guide.h, removed
 *		         	   workaround for endianness
 * 4.00a rkv  02/22/11 Name of the file has been changed for naming consistency
 *		       		   Added interrupt support for Zynq.
 * 5.00a srt  03/05/12 Added Flushing and Invalidation of Caches to fix CRs
 *		       		   648103, 648701.
 *		       		   Added V7 DDR Base Address to fix CR 649405.
 * 6.00a srt  01/24/12 Changed API calls to support MCDMA driver.
 * 7.00a srt  06/18/12 API calls are reverted back for backward compatibility.
 * 7.01a srt  11/02/12 Buffer sizes (Tx and Rx) are modified to meet maximum
 *		       DDR memory limit of the h/w system built with Area mode
 * 7.02a srt  03/01/13 Updated DDR base address for IPI designs (CR 703656).
 * 9.1   adk  01/07/16 Updated DDR base address for Ultrascale (CR 799532) and
 *		       removed the defines for S6/V6.
 * 9.3   ms   01/23/17 Modified xil_printf statement in main function to
 *                     ensure that "Successfully ran" and "Failed" strings are
 *                     available in all examples. This is a fix for CR-965028.
 *       ms   04/05/17 Added tabspace for return statements in functions
 *                     for proper documentation while generating doxygen.
 * 9.6   rsp  02/14/18 Support data buffers above 4GB.Use UINTPTR for storing
 *                     and typecasting buffer address(CR-992638).
 * 9.9   rsp  01/21/19 Fix use of #elif check in deriving DDR_BASE_ADDR.
 * 9.10  rsp  09/17/19 Fix cache maintenance ops for source and dest buffer.
 * 9.14  sk   03/08/22 Delete DDR memory limits comments as they are not
 * 		       relevant to this driver version.
 * 9.15  sa   08/12/22 Updated the example to use latest MIG cannoical define
 * 		       i.e XPAR_MIG_0_C0_DDR4_MEMORY_MAP_BASEADDR.
 * 9.16  sa   09/29/22 Fix infinite loops in the example.
 * </pre>
 *
 * ***************************************************************************
 */
/***************************** Include Files *********************************/
#include "xaxidma.h"
#include "xparameters.h"
#include "xil_exception.h"
#include "xdebug.h"
#include "xil_util.h"

/* TFM Throughput test: read the ARM generic timer directly (CNTPCT_EL0 / CNTFRQ_EL0)
 * instead of depending on xtime_l.h. This is not a lesser substitute - on
 * Cortex-A53 in 64-bit mode, Xilinx's own XTime_GetTime() is itself just a
 * wrapper around a read of CNTPCT_EL0. Doing it directly sidesteps whatever
 * BSP-generation issue is currently keeping xtime_l.h from being found, and
 * has one less header dependency. If/when xtime_l.h starts resolving again,
 * swapping back is a two-line change (these two functions <-> XTime_GetTime).
 */
static inline u64 TfmReadCntpct(void)
{
	u64 val;
	__asm__ __volatile__("mrs %0, cntpct_el0" : "=r" (val));
	return val;
}

static inline u64 TfmReadCntfrq(void)
{
	u64 val;
	__asm__ __volatile__("mrs %0, cntfrq_el0" : "=r" (val));
	return val;
}
#ifdef SDT
#include "xinterrupt_wrap.h"
#endif

#ifdef __aarch64__
#include "xil_mmu.h"
#endif

#ifdef XPAR_UARTNS550_0_BASEADDR
#include "xuartns550_l.h"       /* to use uartns550 */
#endif

#ifndef DEBUG
extern void xil_printf(const char *format, ...);
#endif

#ifndef SDT
#ifdef XPAR_INTC_0_DEVICE_ID
#include "xintc.h"
#else
#include "xscugic.h"
#endif
#endif

/******************** Constant Definitions **********************************/
/*
 * Device hardware build related constants.
 */

#ifndef SDT
#define DMA_DEV_ID		XPAR_AXIDMA_0_DEVICE_ID

#ifdef XPAR_AXI_7SDDR_0_S_AXI_BASEADDR
#define DDR_BASE_ADDR		XPAR_AXI_7SDDR_0_S_AXI_BASEADDR
#elif defined (XPAR_MIG7SERIES_0_BASEADDR)
#define DDR_BASE_ADDR	XPAR_MIG7SERIES_0_BASEADDR
#elif defined (XPAR_MIG_0_C0_DDR4_MEMORY_MAP_BASEADDR)
#define DDR_BASE_ADDR	XPAR_MIG_0_C0_DDR4_MEMORY_MAP_BASEADDR
#elif defined (XPAR_PSU_DDR_0_S_AXI_BASEADDR)
#define DDR_BASE_ADDR	XPAR_PSU_DDR_0_S_AXI_BASEADDR
#endif

#else

#ifdef XPAR_MEM0_BASEADDRESS
#define DDR_BASE_ADDR		XPAR_MEM0_BASEADDRESS
#endif
#endif

#ifndef DDR_BASE_ADDR
#warning CHECK FOR THE VALID DDR ADDRESS IN XPARAMETERS.H, \
DEFAULT SET TO 0x01000000
#define MEM_BASE_ADDR		0x01000000
#else
#define MEM_BASE_ADDR		(DDR_BASE_ADDR + 0x1000000)
#endif

#ifndef SDT
#ifdef XPAR_INTC_0_DEVICE_ID
#define RX_INTR_ID		XPAR_INTC_0_AXIDMA_0_S2MM_INTROUT_VEC_ID
#define TX_INTR_ID		XPAR_INTC_0_AXIDMA_0_MM2S_INTROUT_VEC_ID
#else
#define RX_INTR_ID		XPAR_FABRIC_AXIDMA_0_S2MM_INTROUT_VEC_ID
#define TX_INTR_ID		XPAR_FABRIC_AXIDMA_0_MM2S_INTROUT_VEC_ID
#endif
#endif

#define RX_BD_SPACE_BASE	(MEM_BASE_ADDR)
#define RX_BD_SPACE_HIGH	(MEM_BASE_ADDR + 0x0000FFFF)
#define TX_BD_SPACE_BASE	(MEM_BASE_ADDR + 0x00010000)
#define TX_BD_SPACE_HIGH	(MEM_BASE_ADDR + 0x0001FFFF)

/* TFM Throughput test: buffers sized for a throughput run, not a small correctness
 * check. TX and RX each get TP_BUFFER_BYTES, well apart from each other and
 * from the (unchanged, 64KB each) BD spaces above. Comfortably inside the
 * board's 4GB PS DDR alongside the ELF/heap/stack living below MEM_BASE_ADDR.
 */
#define TP_BUFFER_BYTES		(32 * 1024 * 1024)   /* 32 MiB per run: the
						       * main Phase-3.3 sweep
						       * knob, together with
						       * MAX_PKT_LEN below */
#define TX_BUFFER_BASE		(MEM_BASE_ADDR + 0x00100000)
#define TX_BUFFER_HIGH		(TX_BUFFER_BASE + TP_BUFFER_BYTES - 1)
#define RX_BUFFER_BASE		(TX_BUFFER_BASE + TP_BUFFER_BYTES + 0x00100000)
#define RX_BUFFER_HIGH		(RX_BUFFER_BASE + TP_BUFFER_BYTES - 1)

#ifndef SDT
#ifdef XPAR_INTC_0_DEVICE_ID
#define INTC_DEVICE_ID          XPAR_INTC_0_DEVICE_ID
#else
#define INTC_DEVICE_ID          XPAR_SCUGIC_SINGLE_DEVICE_ID
#endif
#endif

/* Timeout loop counter for reset
 */
#define RESET_TIMEOUT_COUNTER	10000

/*
 * Buffer and Buffer Descriptor related constant definition
 */
#define MARK_UNCACHEABLE        0x701

/* TFM Throughput test: bytes per descriptor - the other main Phase-3.3 sweep knob.
 * BdCountG (computed at runtime from the BD space, see TxSetup) BDs of this
 * size make up ONE single AXI4-Stream frame per run (SOF on BD 0, EOF/TLAST
 * on the last BD) - simpler than the first example's nested multi-packet
 * structure, and enough: framing already proven.
 * With the 64KB BD spaces above and XAXIDMA_BD_MINIMUM_ALIGNMENT = 64B,
 * BdCountG is 1024, so 1024 * MAX_PKT_LEN must equal TP_BUFFER_BYTES.
 */
#define MAX_PKT_LEN		(TP_BUFFER_BYTES / 1024)
#define POLL_TIMEOUT_COUNTER         	5000000U
#define NUMBER_OF_EVENTS		1
/* Coalesce all BDs of one run into a single completion interrupt. BdCountG
 * is only known at runtime, so this is passed as a variable, not a macro -
 * see BdCountG below and its use in TxSetup/RxSetup.
 */
#define DELAY_TIMER_COUNT		100

/* TFM Throughput test: how many timed runs to average over, and a soft cap so a
 * misconfiguration can't wedge the board forever waiting on hardware. */
#define TP_NUM_ITERATIONS	10

/* TFM: simple_dut test configuration ------------------------------------
 * The DUT XORs every 32-bit lane of TDATA with this constant, so at any bus
 * width (32/64/128 bit) each 32-bit word of the RX buffer must equal
 * (TX word XOR DUT_XOR_VALUE).
 * Set DUT_NEGATIVE_TEST to 1 to use a deliberately WRONG reference constant:
 * the run must then FAIL, proving that the checker really detects errors.
 * Stream width is 128 bit (16 bytes): keep MAX_PKT_LEN a multiple of 16 and
 * buffer addresses 16-byte aligned (64 is safer for cache lines).
 */
#define DUT_XOR_VALUE		0xA5A5A5A5U
#define DUT_NEGATIVE_TEST	0
#define RX_POISON_VALUE		0xDEADBEEFU

/* TFM Throughput test: sanity-check the sweep knobs at compile time */
#if (TP_BUFFER_BYTES % 1024) != 0
#error TP_BUFFER_BYTES must be a multiple of 1024 (BdCountG)
#endif
#if (MAX_PKT_LEN % 16) != 0
#error MAX_PKT_LEN must be a multiple of 16 (128-bit stream alignment)
#endif

#ifndef SDT
#ifdef XPAR_INTC_0_DEVICE_ID
#define INTC		XIntc
#define INTC_HANDLER	XIntc_InterruptHandler
#else
#define INTC		XScuGic
#define INTC_HANDLER	XScuGic_InterruptHandler
#endif
#endif

/**************************** Type Definitions *******************************/


/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/
#ifdef XPAR_UARTNS550_0_BASEADDR
static void Uart550_Setup(void);
#endif

static int CheckData(int Length, u8 StartValue);
static void TxCallBack(XAxiDma_BdRing *TxRingPtr);
static void TxIntrHandler(void *Callback);
static void RxCallBack(XAxiDma_BdRing *RxRingPtr);
static void RxIntrHandler(void *Callback);


#ifndef SDT
static int SetupIntrSystem(INTC *IntcInstancePtr,
			   XAxiDma *AxiDmaPtr, u16 TxIntrId, u16 RxIntrId);
static void DisableIntrSystem(INTC *IntcInstancePtr,
			      u16 TxIntrId, u16 RxIntrId);
#endif

static int RxSetup(XAxiDma *AxiDmaInstPtr);
static int RxArm(XAxiDma *AxiDmaInstPtr);
static int TxSetup(XAxiDma *AxiDmaInstPtr);
static int TxSendOnce(XAxiDma *AxiDmaInstPtr);

/************************** Variable Definitions *****************************/
/*
 * Device instance definitions
 */
XAxiDma AxiDma;

#ifndef SDT
static INTC Intc;	/* Instance of the Interrupt Controller */
#endif
/*
 * Flags interrupt handlers use to notify the application context the events.
 */
volatile u32 TxDone;
volatile u32 RxDone;
volatile u32 Error;

/* TFM: framing counters filled by RxCallBack (TLAST -> RXEOF check) */
volatile u32 RxEofCount;	/* RX BDs that carried EOF (= TLAST reached S2MM) */
volatile u32 RxFrameErrors;	/* EOF missing on a last BD, or EOF on a non-last BD */
volatile u32 RxLenErrors;	/* BD transferred bytes != MAX_PKT_LEN */

/* TFM Throughput test: number of BDs actually used for one run, computed once at
 * setup time from the BD space (see TxSetup) and reused everywhere so nothing
 * here depends on a hand-counted constant. */
u32 BdCountG;
u32 CoalesceCountG;   /* IRQThreshold is 8 bits wide (PG021, bits 23:16 of
                        * MM2S_CR/S2MM_CR) - max 255, set once BdCountG is
                        * known, see TxSetup. */
u64 TpIterTicks[TP_NUM_ITERATIONS];     /* elapsed ticks, one per run */
u32   TpIterOk[TP_NUM_ITERATIONS];      /* 1 = CheckData + framing both passed */

/*
 * Buffer for transmit packet. Must be 32-bit aligned to be used by DMA.
 */
u32 *Packet = (u32 *) TX_BUFFER_BASE;

/*****************************************************************************/
/**
*
* Main function
*
* This function is the main entry of the interrupt test. It does the following:
*	- Set up the output terminal if UART16550 is in the hardware build
*	- Initialize the DMA engine
*	- Set up Tx and Rx channels
*	- Set up the interrupt system for the Tx and Rx interrupts
*	- Submit a transfer
*	- Wait for the transfer to finish
*	- Check transfer status
*	- Disable Tx and Rx interrupts
*	- Print test status and exit
*
* @param	None
*
* @return
*		- XST_SUCCESS if tests pass
*		- XST_FAILURE if fails.
*
* @note		None.
*
******************************************************************************/
int main(void)
{
	int Status;
	XAxiDma_Config *Config;
#ifdef SDT
	XAxiDma_BdRing *RxRingPtr;
	XAxiDma_BdRing *TxRingPtr;
#endif

	/* Initial setup for Uart16550 */
#ifdef XPAR_UARTNS550_0_BASEADDR

	Uart550_Setup();

#endif

	xil_printf("\r\n--- Entering main() --- \r\n");
#ifdef __aarch64__
	Xil_SetTlbAttributes(TX_BD_SPACE_BASE, MARK_UNCACHEABLE);
	Xil_SetTlbAttributes(RX_BD_SPACE_BASE, MARK_UNCACHEABLE);
#endif

#ifndef SDT
	Config = XAxiDma_LookupConfig(DMA_DEV_ID);
	if (!Config) {
		xil_printf("No config found for %d\r\n", DMA_DEV_ID);

		return XST_FAILURE;
	}
#else
	Config = XAxiDma_LookupConfig(XPAR_XAXIDMA_0_BASEADDR);
	if (!Config) {
		xil_printf("No config found for %d\r\n", XPAR_XAXIDMA_0_BASEADDR);

		return XST_FAILURE;
	}
#endif
	/* Initialize DMA engine */
	XAxiDma_CfgInitialize(&AxiDma, Config);

	if (!XAxiDma_HasSg(&AxiDma)) {
		xil_printf("Device configured as Simple mode \r\n");
		return XST_FAILURE;
	}

	/* Set up TX/RX channels to be ready to transmit and receive packets */
	Status = TxSetup(&AxiDma);

	if (Status != XST_SUCCESS) {

		xil_printf("Failed TX setup\r\n");
		return XST_FAILURE;
	}

	Status = RxSetup(&AxiDma);
	if (Status != XST_SUCCESS) {

		xil_printf("Failed RX setup\r\n");
		return XST_FAILURE;
	}

	/* Set up Interrupt system  */
#ifndef SDT
	Status = SetupIntrSystem(&Intc, &AxiDma, TX_INTR_ID, RX_INTR_ID);
#else
	TxRingPtr = XAxiDma_GetTxRing(&AxiDma);
	Status = XSetupInterruptSystem(TxRingPtr, &TxIntrHandler,
				       Config->IntrId[0], Config->IntrParent,
				       XINTERRUPT_DEFAULT_PRIORITY);
#endif
	if (Status != XST_SUCCESS) {

		xil_printf("Failed intr setup\r\n");
		return XST_FAILURE;
	}
#ifdef SDT
	RxRingPtr = XAxiDma_GetRxRing(&AxiDma);
	Status = XSetupInterruptSystem(RxRingPtr, &RxIntrHandler,
				       Config->IntrId[1], Config->IntrParent,
				       XINTERRUPT_DEFAULT_PRIORITY);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
#endif

	/* TFM Throughput test: fill the TX pattern and flush it ONCE - the data does
	 * not change between repeat runs, redoing this every iteration
	 * would only add CPU time outside the timed window without changing
	 * the throughput result. word[i] = i, should match CheckData.
	 */
	{
		u32 *TxWords = (u32 *) Packet;
		u32 WordIdx;

		for (WordIdx = 0; WordIdx < TP_BUFFER_BYTES / 4; WordIdx++) {
			TxWords[WordIdx] = WordIdx;
		}
		Xil_DCacheFlushRange((UINTPTR)Packet, TP_BUFFER_BYTES);
	}

	xil_printf("--- Throughput test: %u iterations of %u bytes (%u BDs x %u "
		   "bytes) ---\r\n",
		   (unsigned)TP_NUM_ITERATIONS, (unsigned)TP_BUFFER_BYTES,
		   (unsigned)BdCountG, (unsigned)MAX_PKT_LEN);

	{
		u32 Iter;
		u32 PassCount = 0;

		u64 CountsPerSecond = TfmReadCntfrq();

		xil_printf("Generic timer frequency: %u Hz\r\n",
			   (unsigned)CountsPerSecond);

		for (Iter = 0; Iter < TP_NUM_ITERATIONS; Iter++) {
			u64 TStart, TEnd;

			/* TFM: poison RX fresh each run - a DMA that silently
			 * writes nothing must never read back as a PASS. */
			Xil_DCacheInvalidateRange((UINTPTR)RX_BUFFER_BASE,
						  TP_BUFFER_BYTES);
			{
				u32 *RxWords = (u32 *) RX_BUFFER_BASE;
				u32 WordIdx;
				for (WordIdx = 0; WordIdx < TP_BUFFER_BYTES / 4;
				     WordIdx++) {
					RxWords[WordIdx] = RX_POISON_VALUE;
				}
			}
			Xil_DCacheFlushRange((UINTPTR)RX_BUFFER_BASE,
					     TP_BUFFER_BYTES);

			TxDone = 0;
			RxDone = 0;
			Error = 0;
			RxEofCount = 0;
			RxFrameErrors = 0;
			RxLenErrors = 0;

			/* Re-arm the RX ring: RxCallBack freed last run's BDs
			 * back to the pool but did not reattach buffers. */
			Status = RxArm(&AxiDma);
			if (Status != XST_SUCCESS) {
				xil_printf("Iter %u: RxArm failed\r\n",
					   (unsigned)Iter);
				continue;
			}

			TStart = TfmReadCntpct();

			Status = TxSendOnce(&AxiDma);
			if (Status != XST_SUCCESS) {
				xil_printf("Iter %u: send failed\r\n",
					   (unsigned)Iter);
				continue;
			}

			Status = Xil_WaitForEvent((UINTPTR)&TxDone, BdCountG,
						  BdCountG, POLL_TIMEOUT_COUNTER);
			if (Status != XST_SUCCESS) {
				xil_printf("Iter %u: transmit timeout\r\n",
					   (unsigned)Iter);
				continue;
			}

			Status = Xil_WaitForEvent((UINTPTR)&RxDone, BdCountG,
						  BdCountG, POLL_TIMEOUT_COUNTER);
			if (Status != XST_SUCCESS) {
				xil_printf("Iter %u: receive timeout\r\n",
					   (unsigned)Iter);
				continue;
			}

			TEnd = TfmReadCntpct();
			TpIterTicks[Iter] = TEnd - TStart;

			/* Correctness check happens AFTER the timer stops, so
			 * it never inflates the measured transfer time. */
			Status = CheckData(TP_BUFFER_BYTES, 0);
			if (Status != XST_SUCCESS) {
				xil_printf("Iter %u: data check failed\r\n",
					   (unsigned)Iter);
				TpIterOk[Iter] = 0;
				continue;
			}

			if (RxEofCount != 1 || RxFrameErrors != 0 ||
			    RxLenErrors != 0) {
				xil_printf("Iter %u: framing check failed "
					   "(eof=%u frame_err=%u len_err=%u)"
					   "\r\n", (unsigned)Iter,
					   (unsigned)RxEofCount,
					   (unsigned)RxFrameErrors,
					   (unsigned)RxLenErrors);
				TpIterOk[Iter] = 0;
				continue;
			}

			TpIterOk[Iter] = 1;
			PassCount++;

			{
				double Seconds = (double)TpIterTicks[Iter] /
						 (double)CountsPerSecond;
				double Gbps = ((double)TP_BUFFER_BYTES * 8.0) /
					      Seconds / 1.0e9;

				xil_printf("Iter %u: OK, %u ticks, %d ms, "
					   "throughput (x1000) = %u mGbps\r\n",
					   (unsigned)Iter,
					   (unsigned)TpIterTicks[Iter],
					   (int)(Seconds * 1000.0),
					   (unsigned)(Gbps * 1000.0));
			}
		}

		/* TFM Throughput test: min/mean/max over the PASSING runs only - a
		 * failed run's ticks (if any were even recorded) don't belong
		 * in a throughput number. Printed as milli-Gbps (x1000) since
		 * this BSP's xil_printf has no floating-point %f support. */
		if (PassCount == 0) {
			xil_printf("Throughput test: no passing iterations - "
				   "0/%u\r\n", (unsigned)TP_NUM_ITERATIONS);
			Status = XST_FAILURE;
		} else {
			u64 MinTicks = 0, MaxTicks = 0;
			double SumSeconds = 0.0;
			u32 First = 1;

			for (Iter = 0; Iter < TP_NUM_ITERATIONS; Iter++) {
				if (!TpIterOk[Iter]) {
					continue;
				}
				if (First || TpIterTicks[Iter] < MinTicks) {
					MinTicks = TpIterTicks[Iter];
				}
				if (First || TpIterTicks[Iter] > MaxTicks) {
					MaxTicks = TpIterTicks[Iter];
				}
				SumSeconds += (double)TpIterTicks[Iter] /
					      (double)CountsPerSecond;
				First = 0;
			}

			{
				double MinGbps = ((double)TP_BUFFER_BYTES * 8.0) /
					((double)MaxTicks / (double)CountsPerSecond) /
					1.0e9;
				double MaxGbps = ((double)TP_BUFFER_BYTES * 8.0) /
					((double)MinTicks / (double)CountsPerSecond) /
					1.0e9;
				double MeanGbps = ((double)TP_BUFFER_BYTES * 8.0) *
					(double)PassCount / SumSeconds / 1.0e9;

				xil_printf("Throughput test SUMMARY: %u/%u passed, "
					   "min=%u max=%u mean=%u mGbps (x1000)"
					   "\r\n",
					   (unsigned)PassCount,
					   (unsigned)TP_NUM_ITERATIONS,
					   (unsigned)(MinGbps * 1000.0),
					   (unsigned)(MaxGbps * 1000.0),
					   (unsigned)(MeanGbps * 1000.0));
			}

			Status = (PassCount == TP_NUM_ITERATIONS) ?
				 XST_SUCCESS : XST_FAILURE;
		}
	}

	/* Disable TX and RX Ring interrupts and return */

#ifndef SDT
	DisableIntrSystem(&Intc, TX_INTR_ID, RX_INTR_ID);
#else
	XDisconnectInterruptCntrl(Config->IntrId[0], Config->IntrParent);
	XDisconnectInterruptCntrl(Config->IntrId[1], Config->IntrParent);
#endif

	xil_printf("--- Exiting main() --- \r\n");

	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

#ifdef XPAR_UARTNS550_0_BASEADDR
/*****************************************************************************/
/*
*
* Uart16550 setup routine, need to set baudrate to 9600 and data bits to 8
*
* @param	None
*
* @return	None
*
* @note		None.
*
******************************************************************************/
static void Uart550_Setup(void)
{

	XUartNs550_SetBaud(XPAR_UARTNS550_0_BASEADDR,
			   XPAR_XUARTNS550_CLOCK_HZ, 9600);

	XUartNs550_SetLineControlReg(XPAR_UARTNS550_0_BASEADDR,
				     XUN_LCR_8_DATA_BITS);
}
#endif

/*****************************************************************************/
/*
*
* This function checks data buffer after the DMA transfer is finished.
*
* We use the static tx/rx buffers.
*
* @param	Length is the length to check
* @param	StartValue is the starting value of the first byte
*
* @return	- XST_SUCCESS if validation is successful
*		- XST_FAILURE if validation fails.
*
* @note		None.
*
******************************************************************************/
static int CheckData(int Length, u8 StartValue)
{
	u32 *RxWords = (u32 *) RX_BUFFER_BASE;
	u32 NumWords = (u32)Length / 4;
	u32 Errors = 0;
	u32 Index;
	u32 Expected;
	u32 XorValue = DUT_XOR_VALUE;

	(void)StartValue;	/* TFM: not used, TX data is the word index */

#if DUT_NEGATIVE_TEST
	XorValue = DUT_XOR_VALUE ^ 0x1U;	/* deliberately wrong reference */
	xil_printf("NEGATIVE TEST: using wrong reference constant %08x\r\n",
		   XorValue);
#endif

	/* Invalidate the RX buffer so the CPU reads what the DMA wrote to DDR */
	Xil_DCacheInvalidateRange((UINTPTR)RxWords, Length);

	for (Index = 0; Index < NumWords; Index++) {
		Expected = Index ^ XorValue;
		if (RxWords[Index] != Expected) {
			if (Errors < 10) {
				xil_printf("Data error word %d: got %08x expected %08x\r\n",
					   (int)Index, RxWords[Index], Expected);
			}
			Errors++;
		}
	}

	xil_printf("CheckData: %d words (%d bytes) checked, %d errors\r\n",
		   (int)NumWords, Length, (int)Errors);

	if (Errors != 0) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/*
*
* This is the DMA TX callback function to be called by TX interrupt handler.
* This function handles BDs finished by hardware.
*
* @param	TxRingPtr is a pointer to TX channel of the DMA engine.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void TxCallBack(XAxiDma_BdRing *TxRingPtr)
{
	int BdCount;
	u32 BdSts;
	XAxiDma_Bd *BdPtr;
	XAxiDma_Bd *BdCurPtr;
	int Status;
	int Index;

	/* Get all processed BDs from hardware */
	BdCount = XAxiDma_BdRingFromHw(TxRingPtr, XAXIDMA_ALL_BDS, &BdPtr);

	/* Handle the BDs */
	BdCurPtr = BdPtr;
	for (Index = 0; Index < BdCount; Index++) {

		/*
		 * Check the status in each BD
		 * If error happens, the DMA engine will be halted after this
		 * BD processing stops.
		 */
		BdSts = XAxiDma_BdGetSts(BdCurPtr);
		if ((BdSts & XAXIDMA_BD_STS_ALL_ERR_MASK) ||
		    (!(BdSts & XAXIDMA_BD_STS_COMPLETE_MASK))) {

			Error = 1;
			break;
		}

		/*
		 * Here we don't need to do anything. But if a RTOS is being
		 * used, we may need to free the packet buffer attached to
		 * the processed BD
		 */

		/* Find the next processed BD */
		BdCurPtr = (XAxiDma_Bd *)XAxiDma_BdRingNext(TxRingPtr, BdCurPtr);
	}

	/* Free all processed BDs for future transmission */
	Status = XAxiDma_BdRingFree(TxRingPtr, BdCount, BdPtr);
	if (Status != XST_SUCCESS) {
		Error = 1;
	}

	if (!Error) {

		TxDone += BdCount;
	}
}

/*****************************************************************************/
/*
*
* This is the DMA TX Interrupt handler function.
*
* It gets the interrupt status from the hardware, acknowledges it, and if any
* error happens, it resets the hardware. Otherwise, if a completion interrupt
* presents, then it calls the callback function.
*
* @param	Callback is a pointer to TX channel of the DMA engine.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void TxIntrHandler(void *Callback)
{
	XAxiDma_BdRing *TxRingPtr = (XAxiDma_BdRing *) Callback;
	u32 IrqStatus;
	int TimeOut;

	/* Read pending interrupts */
	IrqStatus = XAxiDma_BdRingGetIrq(TxRingPtr);

	/* Acknowledge pending interrupts */
	XAxiDma_BdRingAckIrq(TxRingPtr, IrqStatus);

	/* If no interrupt is asserted, we do not do anything
	 */
	if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {

		return;
	}

	/*
	 * If error interrupt is asserted, raise error flag, reset the
	 * hardware to recover from the error, and return with no further
	 * processing.
	 */
	if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK)) {

		XAxiDma_BdRingDumpRegs(TxRingPtr);

		Error = 1;

		/*
		 * Reset should never fail for transmit channel
		 */
		XAxiDma_Reset(&AxiDma);

		TimeOut = RESET_TIMEOUT_COUNTER;

		while (TimeOut) {
			if (XAxiDma_ResetIsDone(&AxiDma)) {
				break;
			}

			TimeOut -= 1;
		}

		return;
	}

	/*
	 * If Transmit done interrupt is asserted, call TX call back function
	 * to handle the processed BDs and raise the according flag
	 */
	if ((IrqStatus & (XAXIDMA_IRQ_DELAY_MASK | XAXIDMA_IRQ_IOC_MASK))) {
		TxCallBack(TxRingPtr);
	}
}

/*****************************************************************************/
/*
*
* This is the DMA RX callback function called by the RX interrupt handler.
* This function handles finished BDs by hardware, attaches new buffers to those
* BDs, and give them back to hardware to receive more incoming packets
*
* @param	RxRingPtr is a pointer to RX channel of the DMA engine.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void RxCallBack(XAxiDma_BdRing *RxRingPtr)
{
	int BdCount;
	XAxiDma_Bd *BdPtr;
	XAxiDma_Bd *BdCurPtr;
	u32 BdSts;
	int Index;
	int Status;   /* TFM Throughput test fix: needed for the BdRingFree call below */

	/* Get finished BDs from hardware */
	BdCount = XAxiDma_BdRingFromHw(RxRingPtr, XAXIDMA_ALL_BDS, &BdPtr);

	BdCurPtr = BdPtr;
	for (Index = 0; Index < BdCount; Index++) {

		/*
		 * Check the flags set by the hardware for status
		 * If error happens, processing stops, because the DMA engine
		 * is halted after this BD.
		 */
		BdSts = XAxiDma_BdGetSts(BdCurPtr);
		if ((BdSts & XAXIDMA_BD_STS_ALL_ERR_MASK) ||
		    (!(BdSts & XAXIDMA_BD_STS_COMPLETE_MASK))) {
			Error = 1;
			break;
		}

		/* TFM Throughput test: one run is now a SINGLE frame, so only the very
		 * last BD of the whole run (RxDone == BdCountG - 1) may carry
		 * EOF - TLAST must have arrived exactly there, nowhere else.
		 * Every BD must also have received exactly MAX_PKT_LEN bytes.
		 */
		{
			u32 IsLastOfRun = (RxDone == (BdCountG - 1)) ? 1 : 0;
			u32 HasEof = (BdSts & XAXIDMA_BD_STS_RXEOF_MASK) ? 1 : 0;

			if (HasEof) {
				RxEofCount++;
			}
			if (HasEof != IsLastOfRun) {
				RxFrameErrors++;
			}
			if (XAxiDma_BdGetActualLength(BdCurPtr,
						      RxRingPtr->MaxTransferLen) !=
			    MAX_PKT_LEN) {
				RxLenErrors++;
			}
		}

		/* Find the next processed BD */
		BdCurPtr = (XAxiDma_Bd *)XAxiDma_BdRingNext(RxRingPtr, BdCurPtr);
		RxDone += 1;
	}

	/* TFM Throughput test fix: return the processed BDs to the free pool.
	 * first baseline never did this because it only ran once and never
	 * needed the BDs again; this repeat-loop version does, every
	 * iteration - without it, RxArm() finds FreeBdCount = 0 on every
	 * iteration after the first, which is exactly the failure you saw.
	 */
	Status = XAxiDma_BdRingFree(RxRingPtr, BdCount, BdPtr);
	if (Status != XST_SUCCESS) {
		Error = 1;
	}
}

/*****************************************************************************/
/*
*
* This is the DMA RX interrupt handler function
*
* It gets the interrupt status from the hardware, acknowledges it, and if any
* error happens, it resets the hardware. Otherwise, if a completion interrupt
* presents, then it calls the callback function.
*
* @param	Callback is a pointer to RX channel of the DMA engine.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void RxIntrHandler(void *Callback)
{
	XAxiDma_BdRing *RxRingPtr = (XAxiDma_BdRing *) Callback;
	u32 IrqStatus;
	int TimeOut;

	/* Read pending interrupts */
	IrqStatus = XAxiDma_BdRingGetIrq(RxRingPtr);

	/* Acknowledge pending interrupts */
	XAxiDma_BdRingAckIrq(RxRingPtr, IrqStatus);

	/*
	 * If no interrupt is asserted, we do not do anything
	 */
	if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {
		return;
	}

	/*
	 * If error interrupt is asserted, raise error flag, reset the
	 * hardware to recover from the error, and return with no further
	 * processing.
	 */
	if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK)) {

		XAxiDma_BdRingDumpRegs(RxRingPtr);

		Error = 1;

		/* Reset could fail and hang
		 * NEED a way to handle this or do not call it??
		 */
		XAxiDma_Reset(&AxiDma);

		TimeOut = RESET_TIMEOUT_COUNTER;

		while (TimeOut) {
			if (XAxiDma_ResetIsDone(&AxiDma)) {
				break;
			}

			TimeOut -= 1;
		}

		return;
	}

	/*
	 * If completion interrupt is asserted, call RX call back function
	 * to handle the processed BDs and then raise the according flag.
	 */
	if ((IrqStatus & (XAXIDMA_IRQ_DELAY_MASK | XAXIDMA_IRQ_IOC_MASK))) {
		RxCallBack(RxRingPtr);
	}
}
#ifndef SDT
/*****************************************************************************/
/*
*
* This function setups the interrupt system so interrupts can occur for the
* DMA, it assumes INTC component exists in the hardware system.
*
* @param	IntcInstancePtr is a pointer to the instance of the INTC.
* @param	AxiDmaPtr is a pointer to the instance of the DMA engine
* @param	TxIntrId is the TX channel Interrupt ID.
* @param	RxIntrId is the RX channel Interrupt ID.
*
* @return
*		- XST_SUCCESS if successful,
*		- XST_FAILURE.if not successful
*
* @note		None.
*
******************************************************************************/

static int SetupIntrSystem(INTC *IntcInstancePtr,
			   XAxiDma *AxiDmaPtr, u16 TxIntrId, u16 RxIntrId)
{
	XAxiDma_BdRing *TxRingPtr = XAxiDma_GetTxRing(AxiDmaPtr);
	XAxiDma_BdRing *RxRingPtr = XAxiDma_GetRxRing(AxiDmaPtr);
	int Status;

#ifdef XPAR_INTC_0_DEVICE_ID

	/* Initialize the interrupt controller and connect the ISRs */
	Status = XIntc_Initialize(IntcInstancePtr, INTC_DEVICE_ID);
	if (Status != XST_SUCCESS) {

		xil_printf("Failed init intc\r\n");
		return XST_FAILURE;
	}

	Status = XIntc_Connect(IntcInstancePtr, TxIntrId,
			       (XInterruptHandler) TxIntrHandler, TxRingPtr);
	if (Status != XST_SUCCESS) {

		xil_printf("Failed tx connect intc\r\n");
		return XST_FAILURE;
	}

	Status = XIntc_Connect(IntcInstancePtr, RxIntrId,
			       (XInterruptHandler) RxIntrHandler, RxRingPtr);
	if (Status != XST_SUCCESS) {

		xil_printf("Failed rx connect intc\r\n");
		return XST_FAILURE;
	}

	/* Start the interrupt controller */
	Status = XIntc_Start(IntcInstancePtr, XIN_REAL_MODE);
	if (Status != XST_SUCCESS) {

		xil_printf("Failed to start intc\r\n");
		return XST_FAILURE;
	}

	XIntc_Enable(IntcInstancePtr, TxIntrId);
	XIntc_Enable(IntcInstancePtr, RxIntrId);

#else

	XScuGic_Config *IntcConfig;


	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
				       IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}


	XScuGic_SetPriorityTriggerType(IntcInstancePtr, TxIntrId, 0xA0, 0x3);

	XScuGic_SetPriorityTriggerType(IntcInstancePtr, RxIntrId, 0xA0, 0x3);
	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	Status = XScuGic_Connect(IntcInstancePtr, TxIntrId,
				 (Xil_InterruptHandler)TxIntrHandler,
				 TxRingPtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	Status = XScuGic_Connect(IntcInstancePtr, RxIntrId,
				 (Xil_InterruptHandler)RxIntrHandler,
				 RxRingPtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	XScuGic_Enable(IntcInstancePtr, TxIntrId);
	XScuGic_Enable(IntcInstancePtr, RxIntrId);
#endif

	/* Enable interrupts from the hardware */

	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				     (Xil_ExceptionHandler)INTC_HANDLER,
				     (void *)IntcInstancePtr);

	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function disables the interrupts for DMA engine.
*
* @param	IntcInstancePtr is the pointer to the INTC component instance
* @param	TxIntrId is interrupt ID associated w/ DMA TX channel
* @param	RxIntrId is interrupt ID associated w/ DMA RX channel
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void DisableIntrSystem(INTC *IntcInstancePtr,
			      u16 TxIntrId, u16 RxIntrId)
{
#ifdef XPAR_INTC_0_DEVICE_ID
	/* Disconnect the interrupts for the DMA TX and RX channels */
	XIntc_Disconnect(IntcInstancePtr, TxIntrId);
	XIntc_Disconnect(IntcInstancePtr, RxIntrId);
#else
	XScuGic_Disconnect(IntcInstancePtr, TxIntrId);
	XScuGic_Disconnect(IntcInstancePtr, RxIntrId);
#endif
}
#endif
/*****************************************************************************/
/*
*
* This function sets up RX channel of the DMA engine to be ready for packet
* reception
*
* @param	AxiDmaInstPtr is the pointer to the instance of the DMA engine.
*
* @return	- XST_SUCCESS if the setup is successful.
*		- XST_FAILURE if fails.
*
* @note		None.
*
******************************************************************************/
/* TFM Throughput test: attach fresh buffers to every currently-free RX BD and hand
 * them to hardware. Called once from RxSetup below, and again before each
 * repeat iteration in main(), since RxCallBack frees processed BDs back to
 * the ring but does not re-arm them - the first example never needed to,
 * being one-shot.
 */
static int RxArm(XAxiDma *AxiDmaInstPtr)
{
	XAxiDma_BdRing *RxRingPtr = XAxiDma_GetRxRing(AxiDmaInstPtr);
	XAxiDma_Bd *BdPtr;
	XAxiDma_Bd *BdCurPtr;
	int FreeBdCount;
	UINTPTR RxBufferPtr;
	int Index;
	int Status;

	FreeBdCount = XAxiDma_BdRingGetFreeCnt(RxRingPtr);

	Status = XAxiDma_BdRingAlloc(RxRingPtr, FreeBdCount, &BdPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx bd alloc failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	BdCurPtr = BdPtr;
	RxBufferPtr = RX_BUFFER_BASE;

	for (Index = 0; Index < FreeBdCount; Index++) {

		Status = XAxiDma_BdSetBufAddr(BdCurPtr, RxBufferPtr);
		if (Status != XST_SUCCESS) {
			xil_printf("Rx set buffer addr %x on BD %x failed %d\r\n",
				   (unsigned int)RxBufferPtr,
				   (UINTPTR)BdCurPtr, Status);
			return XST_FAILURE;
		}

		Status = XAxiDma_BdSetLength(BdCurPtr, MAX_PKT_LEN,
					     RxRingPtr->MaxTransferLen);
		if (Status != XST_SUCCESS) {
			xil_printf("Rx set length %d on BD %x failed %d\r\n",
				   MAX_PKT_LEN, (UINTPTR)BdCurPtr, Status);
			return XST_FAILURE;
		}

		/* Receive BDs do not need to set anything for the control
		 * The hardware will set the SOF/EOF bits per stream status
		 */
		XAxiDma_BdSetCtrl(BdCurPtr, 0);
		XAxiDma_BdSetId(BdCurPtr, RxBufferPtr);

		RxBufferPtr += MAX_PKT_LEN;
		BdCurPtr = (XAxiDma_Bd *)XAxiDma_BdRingNext(RxRingPtr, BdCurPtr);
	}

	Status = XAxiDma_BdRingToHw(RxRingPtr, FreeBdCount, BdPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx ToHw failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

static int RxSetup(XAxiDma *AxiDmaInstPtr)
{
	XAxiDma_BdRing *RxRingPtr;
	int Status;
	XAxiDma_Bd BdTemplate;
	int BdCount;

	RxRingPtr = XAxiDma_GetRxRing(&AxiDma);

	/* Disable all RX interrupts before RxBD space setup */
	XAxiDma_BdRingIntDisable(RxRingPtr, XAXIDMA_IRQ_ALL_MASK);

	/* Setup Rx BD space */
	BdCount = XAxiDma_BdRingCntCalc(XAXIDMA_BD_MINIMUM_ALIGNMENT,
					RX_BD_SPACE_HIGH - RX_BD_SPACE_BASE + 1);

	Status = XAxiDma_BdRingCreate(RxRingPtr, RX_BD_SPACE_BASE,
				      RX_BD_SPACE_BASE,
				      XAXIDMA_BD_MINIMUM_ALIGNMENT, BdCount);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx bd create failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	/*
	 * Setup a BD template for the Rx channel. Then copy it to every RX BD.
	 */
	XAxiDma_BdClear(&BdTemplate);
	Status = XAxiDma_BdRingClone(RxRingPtr, &BdTemplate);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx bd clone failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	/*
	 * Set the coalescing threshold. CoalesceCountG (<=128) was computed
	 * in TxSetup (called first) from the 8-bit IRQThreshold limit - a run
	 * of BdCountG BDs now completes over multiple coalesced interrupts
	 * instead of one, which TxDone/RxDone/RxCallBack already handle
	 * correctly regardless of how many batches they arrive in.
	 */
	Status = XAxiDma_BdRingSetCoalesce(RxRingPtr, CoalesceCountG,
					   DELAY_TIMER_COUNT);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx set coalesce failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	/* TFM Throughput test fix: do NOT arm here. RxArm() attaches buffers and
	 * hands them to hardware, consuming every free BD - if we did that
	 * here too, the loop's first RxArm() call in main() would see
	 * FreeBdCount = 0 and XAxiDma_BdRingAlloc() would correctly reject it
	 * with XST_INVALID_PARAM (15), exactly the failure you hit. Arming is
	 * now the loop's job alone, uniformly, for every iteration including
	 * the first - this function only creates and starts the ring. */

	/* Enable all RX interrupts */
	XAxiDma_BdRingIntEnable(RxRingPtr, XAXIDMA_IRQ_ALL_MASK);

	/* Start RX DMA channel */
	Status = XAxiDma_BdRingStart(RxRingPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx start BD ring failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/*
*
* This function sets up the TX channel of a DMA engine to be ready for packet
* transmission.
*
* @param	AxiDmaInstPtr is the pointer to the instance of the DMA engine.
*
* @return	- XST_SUCCESS if the setup is successful.
*		- XST_FAILURE otherwise.
*
* @note		None.
*
******************************************************************************/
static int TxSetup(XAxiDma *AxiDmaInstPtr)
{
	XAxiDma_BdRing *TxRingPtr = XAxiDma_GetTxRing(&AxiDma);
	XAxiDma_Bd BdTemplate;
	int Status;
	u32 BdCount;

	/* Disable all TX interrupts before TxBD space setup */
	XAxiDma_BdRingIntDisable(TxRingPtr, XAXIDMA_IRQ_ALL_MASK);

	/* Setup TxBD space  */
	BdCount = XAxiDma_BdRingCntCalc(XAXIDMA_BD_MINIMUM_ALIGNMENT,
					(UINTPTR)TX_BD_SPACE_HIGH - (UINTPTR)TX_BD_SPACE_BASE + 1);

	/* TFM Throughput test: TX_BD_SPACE and RX_BD_SPACE are the same size with the
	 * same alignment, so this count is valid for both rings; RxSetup reuses
	 * it instead of recomputing. */
	BdCountG = BdCount;
	CoalesceCountG = (BdCountG > 128) ? 128 : BdCountG;
	if (CoalesceCountG == 0) {
		CoalesceCountG = 1;
	}
	if ((u64)BdCountG * (u64)MAX_PKT_LEN != (u64)TP_BUFFER_BYTES) {
		xil_printf("BdCountG (%u) * MAX_PKT_LEN (%u) != TP_BUFFER_BYTES "
			   "(%u) - adjust TP_BUFFER_BYTES so it divides evenly "
			   "by BdCountG\r\n",
			   (unsigned)BdCountG, (unsigned)MAX_PKT_LEN,
			   (unsigned)TP_BUFFER_BYTES);
		return XST_FAILURE;
	}

	Status = XAxiDma_BdRingCreate(TxRingPtr, TX_BD_SPACE_BASE,
				      TX_BD_SPACE_BASE,
				      XAXIDMA_BD_MINIMUM_ALIGNMENT, BdCount);
	if (Status != XST_SUCCESS) {

		xil_printf("Failed create BD ring\r\n");
		return XST_FAILURE;
	}

	/*
	 * Like the RxBD space, we create a template and set all BDs to be the
	 * same as the template. The sender has to set up the BDs as needed.
	 */
	XAxiDma_BdClear(&BdTemplate);
	Status = XAxiDma_BdRingClone(TxRingPtr, &BdTemplate);
	if (Status != XST_SUCCESS) {

		xil_printf("Failed clone BDs\r\n");
		return XST_FAILURE;
	}

	/*
	 * Set the coalescing threshold, so only one transmit interrupt
	 * occurs for this example
	 *
	 * If you would like to have multiple interrupts per run instead,
	 * pass a smaller value than BdCountG here.
	 */
	Status = XAxiDma_BdRingSetCoalesce(TxRingPtr, CoalesceCountG,
					   DELAY_TIMER_COUNT);
	if (Status != XST_SUCCESS) {

		xil_printf("Failed set coalescing"
			   " %d/%d\r\n", (int)CoalesceCountG, DELAY_TIMER_COUNT);
		return XST_FAILURE;
	}

	/* Enable all TX interrupts */
	XAxiDma_BdRingIntEnable(TxRingPtr, XAXIDMA_IRQ_ALL_MASK);

	/* Start the TX channel */
	Status = XAxiDma_BdRingStart(TxRingPtr);
	if (Status != XST_SUCCESS) {

		xil_printf("Failed bd start\r\n");
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/*
*
* This function non-blockingly transmits all packets through the DMA engine.
*
* @param	AxiDmaInstPtr points to the DMA engine instance
*
* @return
* 		- XST_SUCCESS if the DMA accepts all the packets successfully,
* 		- XST_FAILURE if error occurs
*
* @note		None.
*
******************************************************************************/
/* TFM Throughput test: hand BdCountG already-filled BDs to hardware as ONE frame
 * (SOF on BD 0, EOF/TLAST on the last BD). Data fill, RX poisoning and cache
 * flush happen once in main(), not per call, since the data doesn't change
 * between repeat runs and doing them here would sit needlessly inside what
 * would otherwise be the timed window.
 */
static int TxSendOnce(XAxiDma *AxiDmaInstPtr)
{
	XAxiDma_BdRing *TxRingPtr = XAxiDma_GetTxRing(AxiDmaInstPtr);
	XAxiDma_Bd *BdPtr, *BdCurPtr;
	int Status;
	u32 Index;
	UINTPTR BufferAddr;

	/* Per-BD length is validated by XAxiDma_BdSetLength() below, which
	 * fails loudly against TxRingPtr->MaxTransferLen if MAX_PKT_LEN is
	 * too large for this DMA's configured Buffer Length Register width. */
	Status = XAxiDma_BdRingAlloc(TxRingPtr, BdCountG, &BdPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Failed bd alloc\r\n");
		return XST_FAILURE;
	}

	BufferAddr = (UINTPTR)Packet;
	BdCurPtr = BdPtr;

	for (Index = 0; Index < BdCountG; Index++) {
		u32 CrBits = 0;

		Status = XAxiDma_BdSetBufAddr(BdCurPtr, BufferAddr);
		if (Status != XST_SUCCESS) {
			xil_printf("Tx set buffer addr %x on BD %x failed %d\r\n",
				   (unsigned int)BufferAddr,
				   (UINTPTR)BdCurPtr, Status);
			return XST_FAILURE;
		}

		Status = XAxiDma_BdSetLength(BdCurPtr, MAX_PKT_LEN,
					     TxRingPtr->MaxTransferLen);
		if (Status != XST_SUCCESS) {
			xil_printf("Tx set length %d on BD %x failed %d\r\n",
				   MAX_PKT_LEN, (UINTPTR)BdCurPtr, Status);
			return XST_FAILURE;
		}

		if (Index == 0) {
			/* First BD of the whole run: SOF */
			CrBits |= XAXIDMA_BD_CTRL_TXSOF_MASK;
#ifndef SDT
#if (XPAR_AXIDMA_0_SG_INCLUDE_STSCNTRL_STRM == 1)
			Status = XAxiDma_BdSetAppWord(BdCurPtr,
						      XAXIDMA_LAST_APPWORD,
						      MAX_PKT_LEN * BdCountG);
			if (Status != XST_SUCCESS) {
				xil_printf("Set app word failed with %d\r\n", Status);
			}
#endif
#else
			if (TxRingPtr->HasStsCntrlStrm) {
				Status = XAxiDma_BdSetAppWord(BdCurPtr,
							      XAXIDMA_LAST_APPWORD,
							      MAX_PKT_LEN * BdCountG);
				if (Status != XST_SUCCESS) {
					xil_printf("Set app word failed with %d\r\n",
						   Status);
				}
			}
#endif
		}

		if (Index == (BdCountG - 1)) {
			/* Last BD of the whole run: EOF -> TLAST */
			CrBits |= XAXIDMA_BD_CTRL_TXEOF_MASK;
		}

		XAxiDma_BdSetCtrl(BdCurPtr, CrBits);
		XAxiDma_BdSetId(BdCurPtr, BufferAddr);

		BufferAddr += MAX_PKT_LEN;
		BdCurPtr = (XAxiDma_Bd *)XAxiDma_BdRingNext(TxRingPtr, BdCurPtr);
	}

	Status = XAxiDma_BdRingToHw(TxRingPtr, BdCountG, BdPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Failed to hw, length %d\r\n",
			   (int)XAxiDma_BdGetLength(BdPtr, TxRingPtr->MaxTransferLen));
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}