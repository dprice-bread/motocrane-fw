#include <msp430.h> 
#include "stdbool.h"

// Motor control variables
int RXSpeed;
int RXAccel;

int outgoSpeedVal;
int adjustedSpeedVal;
int trueOutgoSpeed;
int realSpeedVal;

int i;
int j;

// Speed parameters
# define SPEED_SHIFT	0 // Bit shift by 0 (x1) to scale between input speed values (out of 100) and output speed values (out of 3200)
# define DEADZONE		1 // -1% <-> -0% & +0% <-> 1% treated as a deadzone and skipped over


const unsigned int HARDMAXACCEL = 20;
const unsigned int MAXACCELINIT = 2;
unsigned int maxAccel;

#define MCLK_TICKS_PER_MS 16000

// Hardware UART variables

unsigned int RXAddress;

unsigned char TXValue[4];	// Can send maximum of 4 bytes
unsigned char TXData[7];		// Total possible bytes to be transmitted = TXValue + 3 header bytes
unsigned char *PTXData = TXData;
unsigned char TXbyteCtr;

unsigned char RXAddressCheck[2]; // Checks for address verification
unsigned char RXNum; // Number of bytes being communicated on RX line
unsigned char RXValue[2];
unsigned char RXbyteCtr;

unsigned char transmitUARTflag; // High when UART transmission is to occur
unsigned char startReceiving; // High when UART receive is starting
#define UCAORX		BIT1	// RX on P1.1
#define UCAOTX		BIT2	// TX on P1.2

//const unsigned int SLAVEADDRESS = 0xa1e2; // For Swing Motor
const unsigned int SLAVEADDRESS_PAN = 0x18f8; // For Pan Motor
const unsigned int SLAVEADDRESS_TILT = 0x474b; // For Tilt Motor
const unsigned int SLAVEADDRESS_ROLL = 0x651f; // For Roll Motor
//const unsigned int SLAVEADDRESS = 0x8c06; // For Linear Motor
//const unsigned int SLAVEADDRESS = 0xf702; // For Lens Motor

const unsigned int CENTRAL_ADDRESS = 0x2782; // The address of this device (for the software UART)
#define UART_DIRECTION BIT0		// Controls the direction of the UART/RS485. Receive is low (default state). Transmit is high

unsigned char select_pan; // 1 when pan motor selected for movement. 0 otherwise
unsigned char select_tilt; // 1 when tilt motor selected for movement. 0 otherwise
unsigned char select_roll; // 1 when roll motor selected for movement. 0 otherwise

// Software UART variables

#define     Bit_time    138	// 115200 Baud, SMCLK=16MHz (16MHz/115200)=138
#define		Bit_time_5	69		// Time for half a bit.

unsigned char TBitCnt;		// Bit count, used for keeping track of bits for RX/TX software UART
unsigned int TXSData[20];		// Total possible ints to be transmitted = 20
unsigned int TXSCounter;
unsigned int TXSByte;		// Value sent over UART when TransmitSoftByte() is called

unsigned char RBitCnt;		// Bit count, used for keeping track of bits for RX/TX software UART
unsigned int RXSData[4];		// Total possible ints to be received = 4
unsigned int RXSCounter;
unsigned int RXSNum;
unsigned int RXSByte;

#define		TXS     BIT0    // TX(Software) on P2.0
#define		RXS		BIT1	// RX(Software) on P2.1


bool isReceiving;		// Status for when the device is receiving
bool hasReceived;		// Lets the program know when a byte is received
bool isTransmitting; 	// Status for when the device is transmitting
bool motorCmdReady; 	// Status for when a command has been received to transmit to the motor

// Reset Timer variables
#define ACLK_TICKS_PER_MS 12
const int RSTDELAY = 150;

// Functions
void initializeMotor(void);
void enableMotor(void);
void setMotorLimits(int);
void setAccel(int);
int mapToDriverSpeed(int);
int correctForDeadzone(int);
void setMotorSpeed(int);
void transmitSoftUART(unsigned int, unsigned int *);
void transmitSoftByte(unsigned int, unsigned int);
void delayMilliseconds(unsigned int);
void transmitCommUART(unsigned int, unsigned char, unsigned char *);


void main(void) {

	delayMilliseconds(1);
	WDTCTL = WDTPW|WDTHOLD; // Disable WDT timer
	delayMilliseconds(50); // If this is shorter, hiccup will be less apparent if motor controller ever resets
	initializeMotor();
	//enableMotor();

	while(1) {

		// Send motor command
		if (motorCmdReady) { // At least 1 command has been received and is waiting for comm to the motor

			// Reset the timer
			TA1CTL = TASSEL_1|MC_0|TACLR;	// ACLK, continuous mode
			TA1CCR0 = ACLK_TICKS_PER_MS*RSTDELAY;	// Set the RST Interval
			TA1R = 0;
			TA1CTL = TASSEL_1|MC_2;	// ACLK, continuous mode

			// Store speed and accel values coming in on Comm UART
			RXSpeed = (int) ((RXValue[0]<<8)>>8); // sign extension
			RXAccel = (int) ((RXValue[1]<<8)>>8); // sign extension

			delayMilliseconds(1);

			// Send out speed, and encoder values on Comm UART
			if (RXNum != 0) { // But only do so if have real data and aren't here due to a timeout
				P1OUT |= UART_DIRECTION; // Set UART_DIRECTION high for transmit
				TXValue[0] = RXValue[0];
				TXValue[1] = (unsigned char) 0;
				TXValue[2] = (unsigned char) 0;
				TXValue[3] = (unsigned char) 0;
				transmitCommUART(CENTRAL_ADDRESS,4,TXValue);
				while (transmitUARTflag == 0x01); // Wait for transmission to complete
			}
			delayMilliseconds(1);
			P1OUT &= ~UART_DIRECTION; // Set UART_DIRECTION low for receive

			//setAccel(RXAccel);
			outgoSpeedVal = mapToDriverSpeed(RXSpeed);
			trueOutgoSpeed = correctForDeadzone(outgoSpeedVal);
			//enableMotor();
			setMotorSpeed(trueOutgoSpeed);

			// Clear Receive Data;
			motorCmdReady = false;
			hasReceived = false;
			RXSpeed = 0;
			RXAccel = MAXACCELINIT;
			RXNum = 0;
			RXValue[0] = 0;
			RXValue[1] = MAXACCELINIT;
			RXbyteCtr = 0;
		}
	}
}

void delayMilliseconds(unsigned int msecs) {
	while (msecs--) {
		__delay_cycles(MCLK_TICKS_PER_MS);
	}
}

void initializeMotor(void) {

	BCSCTL1 = CALBC1_16MHZ; // Adjust the clock to 16MHz operation
	DCOCTL = CALDCO_16MHZ;
	BCSCTL3 = LFXT1S_2; // Select ACLK from VLO. WHY IS THIS HAPPENNING?

	// Setup the Hardware UART
	P1SEL = UCAORX|UCAOTX; // Enable UART function for pin P1.1 and P1.2
	P1SEL2 = UCAORX|UCAOTX;
	UCA0CTL1 |= UCSWRST|UCSSEL_2; // Setup the UART Mode. Enable SW reset, Use SMCLK
	UCA0BR0 = 125; // Low bit of UCRx is 125
	UCA0BR1 = 0; // High bit of UCBRx is 0
	UCA0MCTL = UCBRS_0; // Second modulation stage select is 0. Baud Rate = 128kHz
	UCA0CTL1 &= ~UCSWRST; // Clear SW reset, resume operation
	IE2 |= UCA0RXIE;
	motorCmdReady = false;
	P1DIR = UART_DIRECTION; // Set UART_DIRECTION as output.

	// Setup the Software UART
	P2DIR = TXS;	// Going to manually set/reset the TXS pin rather than use the compare register (Because I can't figure out how)
	P2OUT = TXS;
	P2IES = RXS;	// RXD Hi/lo edge interrupt
	P2IFG = 0x00;
	P2IE = RXS;		// Enable RXD interrupt
	isReceiving = false;	// Set initial values
	hasReceived = false;

	// Set up the reset timer
	TA1CTL = TASSEL_1|MC_0|TACLR;	// ACLK, continuous mode
	TA1CCR0 = ACLK_TICKS_PER_MS*RSTDELAY;	// Set reset timer interval
	TA1R = 0;
	TA1CCTL0 = CCIE;		// Enable interrupts
	TA1CTL = TASSEL_1|MC_2;	// ACLK, continuous mode

	_enable_interrupts();
}

/*
void enableMotor(void) {
	TXSData[0] = 0x83; // Command for error-free start
	transmitSoftUART(1,TXSData);
	while(isTransmitting); // while UART transmission is in progress
	delayMilliseconds(2);
}

void setMotorLimits(int maxAccel) {
	// Set limits for motor operation. These are "soft" limits, so they are lost every time the motor drive is reset.
	TXSData[0] = 0xA2; // Command adjust motor limits
	TXSData[1] = 0x01; // Set max acceleration
	TXSData[2] = (unsigned int) (maxAccel & 0x7F);
	TXSData[3] = (unsigned int) (maxAccel >> 7);
	RXSCounter = 0;
	RXSNum = 1;
	transmitSoftUART(4,TXSData);
	while(isTransmitting); // while UART transmission is in progress
	delayMilliseconds(2);

	TXSData[0] = 0xA2; // Command adjust motor limits
	TXSData[1] = 0x02; // Set max deceleration
	TXSData[2] = (unsigned int) (maxAccel & 0x7F);
	TXSData[3] = (unsigned int) (maxAccel >> 7);
	RXSCounter = 0;
	RXSNum = 1;
	transmitSoftUART(4,TXSData);
	while(isTransmitting); // while UART transmission is in progress
	delayMilliseconds(2);
}

// Sets the acceleration limits of the motor (for both accel and decel)
void setAccel(int accel) {

	if (~(accel==maxAccel)) {
		maxAccel = accel;
		setMotorLimits(maxAccel);
	}
}
*/

// Maps the input speed (from -100 to 100) to a output speed (from -3200 to 3200)
// Maximum speed will be ~1 full revolution per second
int mapToDriverSpeed(int inSpeed) {
	return inSpeed << 5; // Multiple by 32 so that max magnitude is 3200 which is the Pololu driver maximum value
}

// Modifies motor drive range to jump over deadzone
int correctForDeadzone(int currentSpeed) {
	int trueSpeed;
	long tempMemory;

	if (currentSpeed == 0) {
		trueSpeed = 0;

	} else {
		// *** trueSpeed = currentSpeed * ((100<<SPEED_SHIFT)-DEADZONE)/(100<<SPEED_SHIFT) + sgn(currentSpeed)*DEADZONE
		// But we have to use the tempMemory long to maintain precision when working with ints
		tempMemory = (long) currentSpeed * ((100-DEADZONE)<<SPEED_SHIFT);
		trueSpeed = (int) (tempMemory / (100<<SPEED_SHIFT));
		if (currentSpeed < 0) {
			trueSpeed -= (DEADZONE<<SPEED_SHIFT);
		} else
			trueSpeed += (DEADZONE<<SPEED_SHIFT);
	}
	return trueSpeed;
}

// Instructs the Gimbal motor controller to set the operating speed of the motor
void setMotorSpeed(int speed) {
	// Currently hardcoded to only set speed of yaw (AKA pan) axis

	// Command start
	TXSData[0] = 0x3E; // SBGC_CMDN_START_BYTE is the character '>'

	// Command ID
	TXSData[1] = 0x43; // SBGC_CMD_CONTROL is 67 in decimal

	// Data size
	TXSData[2] = 0x0f; // 15 bytes of data

	// Command checksum
	TXSData[3] = 0x52; //Command ID + data size (mod 256)

	// Control mode
	TXSData[4] = 0x01; // SBGC_CMD_CONTROL_MODE_SPEED = 1
	TXSData[5] = 0x01; // SBGC_CMD_CONTROL_MODE_SPEED = 1
	TXSData[6] = 0x01; // SBGC_CMD_CONTROL_MODE_SPEED = 1

	// Roll Speed (2 byte signed)
	if (select_roll == 1) {
		TXSData[7] = (unsigned char) speed & 0x00FF;
		TXSData[8] = (unsigned char) (speed >> 8) & 0x00FF;
	}/* else {
		TXSData[7] = 0x00;
		TXSData[8] = 0x00;
	}*/

	// Roll Angle (2 byte signed)
	TXSData[9] = 0x00;
	TXSData[10] = 0x00;

	// Pitch Speed (2 byte signed)
	if (select_tilt == 1) {
		TXSData[11] = (unsigned char) speed & 0x00FF;
		TXSData[12] = (unsigned char) (speed >> 8) & 0x00FF;
	}/* else {
		TXSData[11] = 0x00;
		TXSData[12] = 0x00;
	}*/

	// Pitch Angle (2 byte signed)
	TXSData[13] = 0x00;
	TXSData[14] = 0x00;

	// Yaw Speed (2 byte signed)
	if (select_pan == 1) {
		TXSData[15] = (unsigned char) speed & 0x00FF;
		TXSData[16] = (unsigned char) (speed >> 8) & 0x00FF;
	}/* else {
		TXSData[15] = 0x00;
		TXSData[16] = 0x00;
	}*/

	// Yaw Angle (2 byte signed)
	// Angle is ignored because speed is assigned and in speed control mode
	TXSData[17] = 0x00;
	TXSData[18] = 0x00;

	// Data checksum
	TXSData[19] = (unsigned char) (TXSData[4]+TXSData[5]+TXSData[6]+TXSData[7]+TXSData[8]+TXSData[9]+TXSData[10]+TXSData[11]+TXSData[12]+TXSData[13]+TXSData[14]+TXSData[15]+TXSData[16]+TXSData[17]+TXSData[18]) & 0x00FF; // sum of data (mod 256)

	transmitSoftUART(20,TXSData);
	while(isTransmitting); // while UART transmission is in progress
	delayMilliseconds(2);
	select_pan = 0;
	select_tilt = 0;
	select_roll - 0;
}

void transmitSoftUART(unsigned int num, unsigned int *pvalue) {

	isTransmitting = true; // Beginning of transmission
	for (i=0; i<num; i++ ) {
		transmitSoftByte(i,num);
	}
}

// Function to Transmit TXByte over software UART
void transmitSoftByte(unsigned int thisbyte, unsigned int num)
{
	TXSByte = TXSData[thisbyte]; // Select the current byte

	while(isReceiving);			// Wait for RX completion
	CCTL0 = OUT;				// TXS Idle as Mark
  	TACTL = TASSEL_2 + MC_2;		// SMCLK, continuous mode

  	TBitCnt = 0xA;				// Load Bit counter, 8 bits + ST/SP
  	CCR0 = TAR;				// Initialize compare register

  	CCR0 += Bit_time;			// Set time till first bit
  	TXSByte |= 0x100;			// Add stop bit to TXByte (which is logical 1)
  	TXSByte = TXSByte << 1;		// Add start bit (which is logical 0)

  	CCTL0 =  CCIS0 + OUTMOD_6 + CCIE;	// Set signal, initial value, enable interrupts
  	while ( CCTL0 & CCIE );			// Wait for previous TX completion

  	// End of transmission
  	if (thisbyte == num-1) {
  		isTransmitting = false;
  	}
}

// Transmits the data in TXData through the UART to the slave at ADDRESS
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
	PTXData = TXData;
	TXbyteCtr = TXData[2]+header+addressLength;
	transmitUARTflag = 0x01;
	IE2 |= UCA0TXIE; // Enable the transmit interrupt
	//UCA1IFG &= ~UCTXIFG; // Clear the UCA1 tx interrupt flags
}

//*** Interupts ***//

// UART transmit interrupt
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void){

	// End condition
	if (TXbyteCtr == 0) {
		IE2 &= ~UCA0TXIE; // Disable the transmit interrupt
		transmitUARTflag = 0x00;
	}

	if (transmitUARTflag == 0x01) { // If transmitting on UART
		UCA0TXBUF = *PTXData;
		PTXData++;
		TXbyteCtr--;
	}
}

// UART receive interrupt (LOOK INTO THIS AGAIN. THERE COULD BE SOURCES OF BUGS HERE)
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void){

	if (startReceiving == 0x01) {
		RXValue[RXbyteCtr++] = UCA0RXBUF;
		if (RXbyteCtr == RXNum) {
			motorCmdReady = true;
			RXbyteCtr = 0;
			startReceiving = 0x00;
		} else if (RXbyteCtr > 5) { // provide a cutoff in case RXNum is incorrect
			RXbyteCtr = 0;
			startReceiving = 0x00;
		}
	}

	// If address is found to match Slave Address, will reset the receive operation
	if (RXAddress == SLAVEADDRESS_PAN) {
		select_pan = 1;
		startReceiving = 0x01;
		RXNum = UCA0RXBUF; // Indicates number of bytes coming in on the channel
		RXbyteCtr = 0;
	}
	if (RXAddress == SLAVEADDRESS_TILT) {
		select_tilt = 1;
		startReceiving = 0x01;
		RXNum = UCA0RXBUF; // Indicates number of bytes coming in on the channel
		RXbyteCtr = 0;
	}
	if (RXAddress == SLAVEADDRESS_ROLL) {
		select_roll = 1;
		startReceiving = 0x01;
		RXNum = UCA0RXBUF; // Indicates number of bytes coming in on the channel
		RXbyteCtr = 0;
	}

	// Rolling monitoring of the address.
	RXAddressCheck[0] = RXAddressCheck[1];
	RXAddressCheck[1] = UCA0RXBUF;
	RXAddress = (unsigned int) (RXAddressCheck[0] << 8) + RXAddressCheck[1];
}

// Incoming signal on software UART interrupt
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
	// If data starts coming in on software UART
	if ((P2IFG & RXS)==RXS) {
		isReceiving = true;

		P2IE &= ~RXS;			// Disable RXD interrupt
		P2IFG &= ~RXS;			// Clear RXD IFG (interrupt flag)

		TACTL = TASSEL_2 + MC_2;	// SMCLK, continuous mode
		CCR0 = TAR;			// Initialize compare register
		CCR0 += Bit_time_5;		// Set time till first bit
		CCTL0 = OUTMOD1 + CCIE;		// Disable TX and enable interrupts

		RXSByte = 0;			// Initialize RXByte
		RBitCnt = 0xa;			// Load Bit counter, 8 bits + ST
	}
	P2IFG = 0x00;
}

// Timer1 A0 interrupt service routine (turns off motor and resets controller if communication from central is lost)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1_A0 (void)
{

	// Turn off motor
	RXSpeed = 0;
	RXAccel = MAXACCELINIT;

	// Hardware UART variables
	PTXData = TXData;
	TXbyteCtr = 0;
	RXbyteCtr = 0;
	transmitUARTflag = 0; // High when UART transmission is to occur
	P1OUT &= ~UART_DIRECTION; // Set UART_DIRECTION low for receive
	select_pan = 0;
	select_tilt = 0;
	select_roll = 0;

	// Software UART variables
	TBitCnt = 0;
	TXSCounter = 0;
	RBitCnt = 0;
	RXSCounter = 0;
	RXSNum = 0;

	// Initialize a command to the motor driver
	isReceiving = false;
	hasReceived = false;
	isTransmitting = false;
	motorCmdReady = true;

	// ESSENTIALLY REPEAT ALL THE INITIALIZATION STUFF //
	// Setup the Hardware UART
	P1SEL = UCAORX|UCAOTX; // Enable UART function for pin P1.1 and P1.2
	P1SEL2 = UCAORX|UCAOTX;
	IE2 |= UCA0RXIE;

	// Setup the Software UART
	P2DIR = TXS;	// Going to manually set/reset the TXS pin rather than use the compare register (Because I can't figure out how)
	P2OUT = TXS;
	P2IES = RXS;	// RXD Hi/lo edge interrupt
	P2IFG = 0x00;
	P2IE = RXS;		// Enable RXD interrupt

	TA1CTL = TASSEL_1|MC_0|TACLR;	// ACLK, continuous mode
	TA1CCR0 = ACLK_TICKS_PER_MS*RSTDELAY;	// Set the RST Interval
	TA1R = 0;
	TA1CTL = TASSEL_1|MC_2;	// ACLK, continuous mode

}


// Timer A0 interrupt service routine (Software UART TX/RX)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0 (void)
{
	// If transmitting
	if(!isReceiving) {
		CCR0 += Bit_time;			// Add Offset to CCR0
		if ( TBitCnt == 0)			// If all bits TXed
		{
			TACTL = TASSEL_2;		// SMCLK, timer off (for power consumption)
			CCTL0 &= ~ CCIE ;		// Disable interrupt
		}
		else
		{
			if (TXSByte & 0x01) {
				P2OUT |= TXS;
			} else {
				P2OUT &= ~TXS;
			}
			TXSByte = TXSByte >> 1;
			TBitCnt --;
		}
	}

	// Receiving
	else {
		CCR0 += Bit_time;				// Add Offset to CCR0

		// In midst of receiving data
		if ((P2IN & RXS) == RXS)		// If bit is set?
			RXSByte |= 0x400;		// Set the value in the RXByte
		RXSByte = RXSByte >> 1;			// Shift the bits down
		RBitCnt --;

		// Finished receiving data
		if ( RBitCnt == 0) {
			TACTL = TASSEL_2;			// SMCLK, timer off (for power consumption)
			CCTL0 &= ~ CCIE ;			// Disable interrupt

			P2IFG &= ~RXS;				// clear RXD IFG (interrupt flag)
			P2IE |= RXS;				// enabled RXD interrupt

			if (RXSCounter > 0) {
				i = 0;
			}

			if ((RXSByte & 0x201) == 0x200)	{	// Validate the start and stop bits are correct
				RXSByte = RXSByte >> 1;		// Remove start bit
				RXSByte &= 0xFF;			// Remove stop bit

				RXSData[RXSCounter++] = RXSByte;

				// After all of the bytes of been received
				if (RXSCounter == RXSNum) {
					RXSCounter = 0;
					RXSNum = 0;
					hasReceived = true;
					isReceiving = false;
				}
			}
		}
	}
}
