#include <msp430.h>
#include "MotoCraneCentral.h" // These are Emmoco-generated Scripts
#include "Hal.h" // This is the Hal integration layer

/* MotoCraneCentral is the firmware that runs on the central MCU of the MotoCrane system. This firmware utilizes the Emmoco BLE stack to communicate with the host app
 *
 * The firmware is set to run in a loop of delayTime ms (currently 50 ms). Every delayTime seconds the processor wakes up, checks the values in ValueArray, then sends the appropriate UART communications signals
 */

/**** BLE Inputs and Values *****/
static int packetSizeVal = 15;
static MotoCraneCentral_packet_t packetVal;
static int statusSizeVal = 11;
static MotoCraneCentral_status_t statusVal;
static int trackSizeVal = 5;
static MotoCraneCentral_track_t trackVal;
int i; // initiate here for usage in outer loops
int j; // initiate here for usage in inner loops
int k; // initiate here for usage in inner loops

const unsigned char stepTime = 25; // 25 ms
const unsigned char delayTime = 50; // 50 ms
unsigned char curTime;
#define MCLK_TICKS_PER_100uS 100

unsigned char ValueArray[15];
unsigned char ValueCheck;

/**** Comm UART Variables and Constants *****/
unsigned char TXValue[2];	// Can send maximum of 2 bytes
unsigned char TXData[5];		// Total possible bytes to be transmitted = TXValue + 1 info byte + 2 address bytes
unsigned char *PTXData;
unsigned char TXbyteCtr;

unsigned char RXAddressCheck[2]; // Checks for address verification
unsigned char RXNum; // Number of bytes being communicated on RX line
unsigned char RXValue[4];
unsigned char RXbyteCtr;
unsigned char RXbyteTotal;
unsigned int RXAddress;

unsigned char transmitUARTflag; // High when UART transmission is to occur
unsigned char receiveUARTflag; // High when UART receive may occur
unsigned char startReceiving; // High when UART receive is starting
#define		TX     BIT4    // TX on P4.4
#define		RX		BIT5	// RX on P4.5
#define UART_DIRECTION BIT0		// P2.0 Controls the direction of the UART/RS485. Receive is low (default state). Transmit is high

/***** Module Addresses *****/
#define CENTRAL_ADDRESS			0x2782 // The address of this device (for the software UART)
#define CAMERA_ADDRESS			0xe9d3
#define SWING_MOTOR_ADDRESS		0xa1e2
#define PAN_MOTOR_ADDRESS		0x18f8
#define TILT_MOTOR_ADDRESS		0x474b
#define LINEAR_MOTOR_ADDRESS	0x8c06
#define ROLL_MOTOR_ADDRESS		0x651f
#define LENS_MOTOR_ADDRESS		0xf702
#define LED0_ADDRESS			0x2859
//#define LED1_ADDRESS			0xd878
//#define LED2_ADDRESS			0xf508


/**** Peripheral Power Control ******/
#define PWR_RELAY BIT1; //P4.1

/**** Functions *****/
static void setupCommUART(void);
static void setupGeneral(void);
static void tickHandler(void);
static void commUART(unsigned int, unsigned char, unsigned char);
static void transmitCommUART(unsigned int, unsigned char, unsigned char *);
static void delayMicroseconds(unsigned int);



void main(void) {
/* MCU first initializes the hall integration layer with Hal_init(). It then initiates its software UART comm.
 * It sets up the repeated loop with Hal_tickStart. Finally it sets up the BLE with MotoCraneCentral_start().
 */
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	Hal_init();
	Hal_tickStart(stepTime, tickHandler);
	MotoCraneCentral_start();
	setupCommUART();
	setupGeneral();
	Hal_idleLoop();

	_enable_interrupts();
}

// Setup the software UART to be ready for TX/RX
static void setupCommUART(void) {

	// Setup the Hardware UART
	P4SEL = RX|TX; // Enable UART function for pin P1.1 and P1.2
	UCA1CTL1 |= UCSWRST|UCSSEL_2; // Setup the UART Mode. Enable SW reset, Use SMCLK
	UCA1BR0 = 8; // Low bit of UCRx is 9
	UCA1BR1 = 0; // High bit of UCBRx is 0
	UCA1MCTL = UCBRS_1; // Second modulation stage select is 1. Baud Rate = 128kHz
	UCA1CTL1 &= ~UCSWRST; // Clear SW reset, resume operation
	P2DIR |= UART_DIRECTION;		// Controls the direction of the UART/RS485. Receive is low (default state). Transmit is high
}

// Set up Power Relay pin to control power to peripherals
static void setupGeneral(void) {

	// Enable peripheral power
	P4DIR |= PWR_RELAY;
	P4OUT |= PWR_RELAY;
}

static void tickHandler(void) {

	// Only do something if delayVal ms have passed
	curTime += stepTime;
	if (curTime < delayTime) {
		return;
	}
	curTime = 0; // reset time value

	// reset comm variables
	PTXData = TXData;
	TXbyteCtr = 0;
	RXbyteCtr = 0;
	RXbyteTotal = 0;
	transmitUARTflag = 0; // High when UART transmission is to occur
	receiveUARTflag = 0; // High when UART receival is to occur
	UCA1IE &= ~(UCTXIE|UCRXIE); // Disable the transmit interrupt

	for (i = 0; i < 15; i++) {
		ValueCheck = packetVal[i];

		if (i == 0) { // Camera Control
			if (ValueCheck != ValueArray[i]) {
				ValueArray[i] = ValueCheck; // Update ValueArray with most recent value

				TXValue[0] = packetVal[i]; // Camera LANC command
				commUART(CAMERA_ADDRESS,1,0);
			}

		} else if (i == 1) { // Power for Noncentral circuits (motor controllers, camera control, LED control)
			ValueArray[i] = ValueCheck; // Update ValueArray with most recent value
			if (ValueCheck == 1) { // Latch Motor Control
				P4OUT &= ~PWR_RELAY;
			} else {
				P4OUT |= PWR_RELAY;
			}

		} else if ((i == 2)|(i == 4)|(i == 6)|(i == 8)|(i == 10)|(i == 12)) { // Motor Control
			if ((ValueCheck != ValueArray[i]) | (ValueCheck != 0) | ((ValueCheck == 0) & (statusVal[i/2+1] != 0))) { // [i/2+1] is the indice in statusVal that corresponds to indice [i] of ValueArray
				ValueArray[i] = ValueCheck; // Update ValueArray with most recent value

				TXValue[0] = packetVal[i]; // Motor Speed
				TXValue[1] = packetVal[i+1]; // Motor Accel
				if (i == 2) { // Swing Motor Control
					commUART(SWING_MOTOR_ADDRESS,2,4);
				} else if (i == 4) { // Pan Motor Control
					commUART(PAN_MOTOR_ADDRESS,2,4);
				} else if (i == 6) { // Tilt Motor Control
					commUART(TILT_MOTOR_ADDRESS,2,4);
				} else if (i == 8) { // Linear Motor Control
					commUART(LINEAR_MOTOR_ADDRESS,2,4);
				} else if (i == 10) { // Roll Motor Control
					commUART(ROLL_MOTOR_ADDRESS,2,4);
				} else if (i == 12) { // Lens Motor Control
					commUART(LENS_MOTOR_ADDRESS,2,4);
				}
			}

		} else if (i == 14) { // LED Control
			if (ValueCheck != ValueArray[i]) {
				ValueArray[i] = ValueCheck; // Update ValueArray with most recent value

				TXValue[0] = packetVal[i]; // LED command
				if (i == 14) { // LED Control
					commUART(LED0_ADDRESS,1,0);
				}
			}
		}
	}
}

// Initiates the communication on the UART channels. After transmitting, waits for receive data and records it
static void commUART(unsigned int address, unsigned char numTX, unsigned char numRX) {

	// First transmit the data
	P2OUT |= UART_DIRECTION; // Set UART_DIRECTION high for transmit
	RXbyteTotal = numRX;
	transmitCommUART(address, numTX, TXValue);
	while (transmitUARTflag == 0x01); // Wait for transmission to complete
	delayMicroseconds(100);

	P2OUT &= ~UART_DIRECTION; // Set UART_DIRECTION low for receive
	delayMicroseconds(3000); // Should be enough time to receive the data

	// Second, if there is any data to receive, record it
	if (numRX > 0) {

		// Assign the received data to the appropriate indices in statusVal and trackVal vectors
		trackVal[0] = RXValue[1];
		if (i==2) {
			trackVal[1] = (int) (RXValue[2] << 8) + RXValue[3];
			statusVal[2] = RXValue[0];
		} else if (i==4) {
			trackVal[2] = (int) (RXValue[2] << 8) + RXValue[3];
			statusVal[3] = RXValue[0];
		} else if (i==6) {
			trackVal[3] = (int) (RXValue[2] << 8) + RXValue[3];
			statusVal[4] = RXValue[0];
		} else if (i==8) {
			trackVal[4] = (int) (RXValue[2] << 8) + RXValue[3];
			statusVal[5] = RXValue[0];
		} else if (i==10) {
			//trackVal[5] = ?? // THERE IS NO TRACKVAL[5] as schema is locked. SO ROLL IS CURRENTLY NOT RECORDED INTO TRACKVAL
			statusVal[6] = RXValue[0];
		} else if (i==12) {
			//trackVal[6] = ?? // THERE IS NO TRACKVAL[6] as schema is locked. SO ROLL IS CURRENTLY NOT RECORDED INTO TRACKVAL
			statusVal[7] = RXValue[0];
		}

		RXValue[0] = 0;
		//RXValue[1] = 0; Don't reset this one as it is voltage
		RXValue[2] = 0;
		RXValue[3] = 0;
	}
}

// Transmits the data in TXData through UART to the slave at ADDRESS
static void transmitCommUART(unsigned int address, unsigned char num, unsigned char *pvalue) {

	unsigned char header = 1; // indicates 1 bytes that will be the header of the signal
	unsigned char addressLength = 2; // 2 bytes in the address

	// Set up the transmission vector
	TXData[0] = (unsigned char) (address >> 8) & 0x00FF;
	TXData[1] = (unsigned char) address & 0x00FF;
	TXData[2] = num;
	for (j=0; j<num; j++)
		TXData[j+header+addressLength] = *(pvalue+j);

	// Enable the UART transmission
	startReceiving = 0x00;
	PTXData = TXData;
	TXbyteCtr = TXData[2]+header+addressLength;
	transmitUARTflag = 0x01;
	receiveUARTflag = 0x00;
	UCA1IE &= ~UCRXIE; // Disable the receive interrupt
	UCA1IE |= UCTXIE|UCRXIE; // Enable the transmit interrupt
	//UCA1IFG &= ~UCTXIFG; // Clear the UCA1 tx interrupt flags
}

void delayMicroseconds(unsigned int msecs) {
	while (msecs>0) {
		msecs-= 100;
		__delay_cycles(MCLK_TICKS_PER_100uS);
	}
}

// UART transmit/receive interrupt
#pragma vector= USCI_A1_VECTOR
__interrupt void USCI_A1ISR(void){

	// End condition (or repeated start in event of receive)
	if ((TXbyteCtr == 0) & (receiveUARTflag == 0x00)) {
		UCA1IE &= ~UCTXIE; // Disable the transmit interrupt
		transmitUARTflag = 0x00;
		if (RXbyteTotal!=0) {
			receiveUARTflag = 0x01;
		}


	} else if (transmitUARTflag == 0x01) { // If transmitting on UART
		UCA1TXBUF = *PTXData;
		PTXData++;
		TXbyteCtr--;

	} else if (receiveUARTflag == 0x01) { // If receiving on UART

		if (startReceiving == 0x01) {
			RXValue[RXbyteCtr++] = UCA1RXBUF;
			if (RXbyteCtr == RXNum) {
				UCA1IE &= ~UCRXIE; // Disable the receive interrupt
				receiveUARTflag = 0x00;
				RXbyteCtr = 0;
				RXNum = 0;
				startReceiving = 0x00;
			} else if (RXbyteCtr > 6) { // provide a cutoff in case RXNum is incorrect
				UCA1IE &= ~UCRXIE; // Disable the receive interrupt
				RXbyteCtr = 0;
				receiveUARTflag = 0x00;
				startReceiving = 0x00;
				RXNum = 0;
				RXValue[0] = 0;
				RXValue[1] = 0;
				RXValue[2] = 0;
				RXValue[3] = 0;
			}
		}

		// If address is found to match Central_Address, will reset the receive operation
		if (RXAddress == CENTRAL_ADDRESS) {
			startReceiving = 0x01;
			RXNum = UCA1RXBUF; // Indicates number of bytes coming in on the channel
			RXbyteCtr = 0;
		}

		// Rolling monitoring of the address.
		RXAddressCheck[0] = RXAddressCheck[1];
		RXAddressCheck[1] = UCA1RXBUF;
		RXAddress = (unsigned int) (RXAddressCheck[0] << 8) + RXAddressCheck[1];
	}
}


/* -------- SCHEMA CALLBACKS -------- */
/* These are the communication functions that interface with the MotoCrane app*/

void MotoCraneCentral_connectHandler(void) {
	Hal_connected();
}

void MotoCraneCentral_disconnectHandler(void) {
	Hal_disconnected();
}

void MotoCraneCentral_packet_store(MotoCraneCentral_packet_t input) {
    for (i = 0; i < packetSizeVal; i++) {
        packetVal[i] = input[i];
    }
}

void MotoCraneCentral_status_fetch(MotoCraneCentral_status_t output) {
    for (i = 0; i < statusSizeVal; i++) {
        output[i] = statusVal[i];
    }
}

void MotoCraneCentral_track_fetch(MotoCraneCentral_track_t output) {
    for (i = 0; i < trackSizeVal; i++) {
        output[i] = trackVal[i];
    }
}


