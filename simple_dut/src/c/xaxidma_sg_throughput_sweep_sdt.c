#include "xaxidma.h"
#include "xparameters.h"
#include "xil_exception.h"
#include "xdebug.h"
#include "xil_util.h"

/* Throughput test: read the ARM generic timer directly (CNTPCT_EL0 / CNTFRQ_EL0)
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
#include "xinterrupt_wrap.h"

#ifndef XPAR_XAXIDMA_0_BASEADDR
#error XPAR_XAXIDMA_0_BASEADDR is not defined - this file targets the SDT flow (Vitis Unified IDE).
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


/******************** Constant Definitions **********************************/
/*
 * Device hardware build related constants.
 */


#ifdef XPAR_MEM0_BASEADDRESS
#define DDR_BASE_ADDR		XPAR_MEM0_BASEADDRESS
#endif

#ifndef DDR_BASE_ADDR
#warning CHECK FOR THE VALID DDR ADDRESS IN XPARAMETERS.H, \
DEFAULT SET TO 0x01000000
#define MEM_BASE_ADDR		0x01000000
#else
#define MEM_BASE_ADDR		(DDR_BASE_ADDR + 0x1000000)
#endif


/* TFM Throughput: the two knobs to actually sweep. Change either one and
 * rebuild - everything else on this page (BD space size, buffer placement)
 * is computed FROM these two, so no combination needs hand-adjusted offsets.
 * Suggested sweep points, in priority order, each with everything else held
 * fixed:
 *   TP_BYTES_PER_BD : 256, 1024, 4096, 8192, 32768 (current), 131072, 1048576
 *     - Sandia/UNM (arXiv:2404.10619) found SG-DMA throughput matches
 *       simple-mode once buffers exceed 2^12 (4096) bytes - expect the
 *       interesting behaviour, if any, BELOW that, not above it.
 *   TP_BUFFER_BYTES : 1M, 4M, 32M (current), 128M - holding TP_BYTES_PER_BD
 *     fixed while this varies changes ring depth (BD count) directly.
 * For BD sizes below ~4KB, TP_NUM_BDS_NEEDED gets large (32MB/256B =
 * 131072 BDs) - the per-BD C setup loop still runs outside the timed
 * window so it can't corrupt the throughput number, but it adds real
 * wall-clock time per iteration. Dropping TP_BUFFER_BYTES to 1-4MB for just
 * the smallest BD-size points keeps each run quick without changing what's
 * being measured.
 */
#define TP_BYTES_PER_BD		(32 * 1024)
#define TP_BUFFER_BYTES		(32 * 1024 * 1024)
#define TP_NUM_BDS_NEEDED	(TP_BUFFER_BYTES / TP_BYTES_PER_BD)

/* IRQThreshold is an 8-bit field (PG021, bits 23:16) - max 255 - and it counts
 * PACKETS, i.e. descriptors with the EOF bit set (PG021 DMASR.IOC_Irq and the
 * driver docs: "packet counter threshold"), NOT individual BDs.
 * This file sends ONE packet per run (EOF only on the last BD), so a value
 * above 1 is never reached: the interrupt is raised by the DELAY timer
 * instead (DELAY_TIMER_COUNT x 125 x SG-clock period after the end of the
 * packet = 62.5 us at 200 MHz with the value 100). Sweeping this macro has
 * NO effect until a run is split into several packets (several EOF BDs).
 */
#define TP_COALESCE_CAP		128

/* 0x40 = XAXIDMA_BD_MINIMUM_ALIGNMENT; TxSetup's runtime self-check
 * catches it if that assumption is ever wrong on real hardware. */
#define TP_BD_SPACE_BYTES	((u64)TP_NUM_BDS_NEEDED * 0x40)

#define RX_BD_SPACE_BASE	(MEM_BASE_ADDR)
#define RX_BD_SPACE_HIGH	(RX_BD_SPACE_BASE + TP_BD_SPACE_BYTES - 1)
#define TX_BD_SPACE_BASE	((RX_BD_SPACE_HIGH + 1 + 0xFFF) & ~0xFFFULL)
#define TX_BD_SPACE_HIGH	(TX_BD_SPACE_BASE + TP_BD_SPACE_BYTES - 1)

#define TX_BUFFER_BASE		((TX_BD_SPACE_HIGH + 1 + 0xFFFFFULL) & ~0xFFFFFULL)
#define TX_BUFFER_HIGH		(TX_BUFFER_BASE + TP_BUFFER_BYTES - 1)
#define RX_BUFFER_BASE		((TX_BUFFER_HIGH + 1 + 0xFFFFFULL) & ~0xFFFFFULL)
#define RX_BUFFER_HIGH		(RX_BUFFER_BASE + TP_BUFFER_BYTES - 1)


/* Timeout loop counter for reset
 */
#define RESET_TIMEOUT_COUNTER	10000

/*
 * Buffer and Buffer Descriptor related constant definition
 */
#define MARK_UNCACHEABLE        0x701

// One single AXI4-Stream frame per run (SOF on BD 0, EOF/TLAST on the last BD)
#define MAX_PKT_LEN		TP_BYTES_PER_BD
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

#if (TP_BUFFER_BYTES % TP_BYTES_PER_BD) != 0
#error TP_BUFFER_BYTES must be an exact multiple of TP_BYTES_PER_BD
#endif
#if (TP_BYTES_PER_BD % 16) != 0
#error TP_BYTES_PER_BD must be a multiple of 16 (128-bit stream alignment)
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



static int RxSetup(XAxiDma *AxiDmaInstPtr);
static int RxArm(XAxiDma *AxiDmaInstPtr);
static int TxSetup(XAxiDma *AxiDmaInstPtr);
static int TxSendOnce(XAxiDma *AxiDmaInstPtr);

/************************** Variable Definitions *****************************/
/*
 * Device instance definitions
 */
XAxiDma AxiDma;

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

// TFM: variables for number of BDs and coalescing
u32 BdCountG;
u32 CoalesceCountG;   /* IRQThreshold: 8-bit field (PG021, bits 23:16), counts
                        * PACKETS (EOF descriptors), max 255. See TP_COALESCE_CAP. */
u64 TpIterTicks[TP_NUM_ITERATIONS];	/* elapsed ticks, one per run */
u32 TpIterOk[TP_NUM_ITERATIONS];    /* 1 = CheckData + framing both passed */

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
	XAxiDma_BdRing *RxRingPtr;
	XAxiDma_BdRing *TxRingPtr;

	/* Initial setup for Uart16550 */
#ifdef XPAR_UARTNS550_0_BASEADDR

	Uart550_Setup();

#endif

	xil_printf("\r\n--- Entering main() --- \r\n");
#ifdef __aarch64__
	/* Xil_SetTlbAttributes() changes a whole 2 MB section for addresses below
	 * 4 GB (UG643). The BD spaces now scale with TP_NUM_BDS_NEEDED and can span
	 * several sections, so mark EVERY section from the start of the RX BD space
	 * to the end of the TX BD space as non-cacheable. (Marking only the first
	 * address of each space left the rest of a large BD space cacheable, and the
	 * CPU/DMA would then disagree about the descriptors' contents.) */
	{
		UINTPTR TlbAddr;

		for (TlbAddr = (UINTPTR)(RX_BD_SPACE_BASE & ~0x1FFFFFULL);
		     TlbAddr <= (UINTPTR)TX_BD_SPACE_HIGH;
		     TlbAddr += 0x200000ULL) {
			Xil_SetTlbAttributes(TlbAddr, MARK_UNCACHEABLE);
		}
	}
#endif

	Config = XAxiDma_LookupConfig(XPAR_XAXIDMA_0_BASEADDR);
	if (!Config) {
		xil_printf("No config found for %d\r\n", XPAR_XAXIDMA_0_BASEADDR);

		return XST_FAILURE;
	}
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
	TxRingPtr = XAxiDma_GetTxRing(&AxiDma);
	Status = XSetupInterruptSystem(TxRingPtr, &TxIntrHandler,
				       Config->IntrId[0], Config->IntrParent,
				       XINTERRUPT_DEFAULT_PRIORITY);
	if (Status != XST_SUCCESS) {

		xil_printf("Failed intr setup\r\n");
		return XST_FAILURE;
	}
	RxRingPtr = XAxiDma_GetRxRing(&AxiDma);
	Status = XSetupInterruptSystem(RxRingPtr, &RxIntrHandler,
				       Config->IntrId[1], Config->IntrParent,
				       XINTERRUPT_DEFAULT_PRIORITY);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* TFM Throughput test: fill the TX pattern and flush it ONCE - the data does
	 * not change between repeat runs, so redoing this every iteration
	 * would only add CPU time outside the timed window without changing
	 * the throughput result. word[i] = i, matching example's CheckData.
	 */
	{
		u32 *TxWords = (u32 *) Packet;
		u32 WordIdx;

		for (WordIdx = 0; WordIdx < TP_BUFFER_BYTES / 4; WordIdx++) {
			TxWords[WordIdx] = WordIdx;
		}
		Xil_DCacheFlushRange((UINTPTR)Packet, TP_BUFFER_BYTES);
	}

	xil_printf("--- Throughput test: %u iterations of %u bytes (%u BDs x %u bytes) ---\r\n",
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

			/* NOTE on what this window contains: TxSendOnce() below prepares ALL
			 * the TX descriptors in software (BdCountG BDs, written to
			 * non-cacheable memory) BEFORE it writes the tail pointer that starts
			 * the hardware, so that CPU work is inside the measured time. It
			 * grows with the number of BDs, so small-BD sweep points also measure
			 * CPU descriptor preparation, not only the DMA. Not yet measured
			 * separately; hw should have a clean way to isolate the hardware-only time. */
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

			// Check data after the timer stops
			Status = CheckData(TP_BUFFER_BYTES, 0);
			if (Status != XST_SUCCESS) {
				xil_printf("Iter %u: data check failed\r\n",
					   (unsigned)Iter);
				TpIterOk[Iter] = 0;
				continue;
			}

			if (RxEofCount != 1 || RxFrameErrors != 0 || RxLenErrors != 0) {
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

	XDisconnectInterruptCntrl(Config->IntrId[0], Config->IntrParent);
	XDisconnectInterruptCntrl(Config->IntrId[1], Config->IntrParent);

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

	/* TFM Throughput test: return the processed BDs to the free pool.
	 * first baseline never did this because it only ran once and never
	 * needed the BDs again; this repeat-loop version does, every
	 * iteration - without it, RxArm() finds FreeBdCount = 0 on every
	 * iteration after the first.
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
	 * Set the coalescing threshold. The threshold counts PACKETS (EOF
	 * descriptors), not BDs. This file sends one packet per run, so the value
	 * is never reached and the interrupt comes from the delay timer
	 * (DELAY_TIMER_COUNT) after the end of the packet - see TP_COALESCE_CAP.
	 * RxCallBack/TxCallBack process however many BDs are finished when it fires.
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
	CoalesceCountG = (BdCountG > TP_COALESCE_CAP) ? TP_COALESCE_CAP : BdCountG;
	if (CoalesceCountG == 0) {
		CoalesceCountG = 1;
	}
	if ((u64)BdCountG * (u64)MAX_PKT_LEN != (u64)TP_BUFFER_BYTES) {
		xil_printf("BdCountG (%u) * MAX_PKT_LEN (%u) != TP_BUFFER_BYTES "
			   "(%u) - TP_BUFFER_BYTES must be an exact multiple of "
			   "TP_BYTES_PER_BD\r\n",
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
	 * Set the coalescing threshold. It counts PACKETS (EOF descriptors), not
	 * BDs; with one packet per run the transmit interrupt is raised by the
	 * delay timer. To get several interrupts per run, split the run into
	 * several packets (EOF on every N-th BD) - see TP_COALESCE_CAP.
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
			if (TxRingPtr->HasStsCntrlStrm) {
				Status = XAxiDma_BdSetAppWord(BdCurPtr,
							      XAXIDMA_LAST_APPWORD,
							      MAX_PKT_LEN * BdCountG);
				if (Status != XST_SUCCESS) {
					xil_printf("Set app word failed with %d\r\n",
						   Status);
				}
			}
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
