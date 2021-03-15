//  Interfacing ADXL345 accelerometer with MSP430FR2355 using I2C communication. XBee radio module is interfaced with UART.
//  Data is transmitted wirelessly to XCTU using UART A0 or to a terminal in Code Composer Studio (by way of micro USB) using UART A1.
//                                   3V3
//                                                                                XBee Transceiver
//                  ADXL345                         MSP430FR2355                      -------     ^)))
//                   Slave                             Master                        /       \____|
//             -----------------           ----------------------------             /         \
//            |3V3           SDA|<------+>|P1.2/UCB0SDA    P1.7/UCA0TXD|---------->|DIN     3V3|
//            |                 |         |                            |           |           |
//            |                 |         |                            |           |           |
//            |GND           SCL|<------->|P1.3/UCB0SCL    P4.3/UCA1RXD|<----------|DOUT    GND|
//            |                 |         |3V3                      GND|           |           |
//----------------------------------------------------------------------------------------------------------
#include <msp430.h> 
#include <math.h>               // Need for M_PI
#include <string.h>             // Need for strlen
#include <stdio.h>              // Need for sprintf

struct{                         // Structure for converting floating point values to char array to transmit over UART one byte at a time
char Buffer[150];
volatile char* pBuf;
volatile unsigned char Length;
}eUSCI_A0_UART;

#define NUM_BYTES_TX 2                  // Number of bytes being transmitted (Transmitting an address to write to and the value being written to that address
#define NUM_BYTES_RX 6                  // Number of bytes being received (Reading 6 acceleration registers)
#define ADXL_345 0x53                   // 7 bit address of the ADXL345

volatile unsigned char RxBuffer[6];     // Allocate 6 bytes of RAM. Each register is 8 bits and holds the output data for each axis. Two acceleration registers for each axis.
unsigned char *PRxData;                 // Pointer to RX data
unsigned char MSData[2];                // Used to hold register address and data for transmitting to ADXL345
unsigned char TXByteCtr;
float x_raw, y_raw, z_raw;              // Raw acceleration value from bit shifting
float x, y, z;                          // Acceleration value in g after dividing by scaling factor
float pitch, roll;                      // Tilt angles

//-- Function prototypes I2C
void Setup_TX(unsigned char Dev_Addr);
void Setup_RX(unsigned char Dev_Addr);
void Transmit(unsigned char Reg_Addr, unsigned char Reg_Data);
void TransmitOne(unsigned char Reg_Addr);
void Receive(void);

// Functions prototypes UART
void Setup_UART();
void Setup_UART_Rx();
void UCA0_UART_Send(float x, float y, float z, float roll, float pitch);

//-- Register Variables
unsigned char DATA_FORMAT = 0x31;        // Set up range -> +-2g, +-4g, +-8g
unsigned char POWER_CTL = 0x2D;          // To place ADXL345 into measurement mode
unsigned char DATAX0 = 0x32;             // X-Axis Data address. ADXL345 will auto-increment to read following acceleration registers, 0x32 -> 0x37
unsigned char BW_RATE = 0x2C;            // Set output data rate (100 Hz by default)
unsigned char OFSX = 0x1E;               // X-axis offset register
unsigned char OFSY = 0x1F;               // Y-axis offset register
unsigned char OFSZ = 0X20;               // Z-axis offset register

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;            // stop watchdog timer

    //-- Configure ports:
    //-- ADXL345 I2C pins
    P1SEL1 &= ~BIT3;                     // P1.3 = SCL
    P1SEL0 |= BIT3;

    P1SEL1 &= ~BIT2;                     // P1.2 = SDA
    P1SEL0 |= BIT2;

    Setup_TX(ADXL_345);                     // Configure I2C to transmit data
    Transmit(DATA_FORMAT, 0x00);            // +-2g by default
    while( ( UCB0IFG & UCSTPIFG ) == 0){}   // Only time this is not zero is when the STOP flag is asserted
                                                // and condition becomes false and leave while loop. Polling STOP flag to wait for the message to be done
    UCB0IFG &= ~UCSTPIFG;                   // Clear the stop flag

    /*                                      // Remove to change output data rate
    Setup_TX(ADXL_345);
    Transmit(BW_Rate, 0x0A);                // Default value is 0x0A which translates to 100 Hz output data rate
    while( ( UCB0IFG & UCSTPIFG ) == 0){}
    UCB0IFG &= ~UCSTPIFG;                   // Clear the stop flag
    */

    Setup_TX(ADXL_345);
    Transmit(POWER_CTL, 0x08);              // Setting of 1 in measurement bit places the part into measurement mode
    while( ( UCB0IFG & UCSTPIFG ) == 0){}
    UCB0IFG &= ~UCSTPIFG;                   // Clear the stop flag

    Setup_TX(ADXL_345);                     // Offset registers are used for calibration
    Transmit(OFSX, 0);                      // Writes 0 to x-axis offset register in ADXL345. This value is automatically added to raw acceleration value
    while( ( UCB0IFG & UCSTPIFG ) == 0){}
    UCB0IFG &= ~UCSTPIFG;                   // Clear the stop flag

    Setup_TX(ADXL_345);
    Transmit(OFSY, -3);                     // Writes -3 to y-axis offset register in ADXL345. This value is automatically added to raw acceleration value
    while( ( UCB0IFG & UCSTPIFG ) == 0){}
    UCB0IFG &= ~UCSTPIFG;                   // Clear the stop flag

    Setup_TX(ADXL_345);
    Transmit(OFSZ, 2);                      // Writes 1 to z-axis offset register in ADXL345. This value is automatically added to raw acceleration value
    while( ( UCB0IFG & UCSTPIFG ) == 0){}
    UCB0IFG &= ~UCSTPIFG;                   // Clear the stop flag

    Setup_UART_Rx();                        // Puts MSP430 into LPM until start frame from XCTU

    while(1)
    {
        // Transmit process
        Setup_TX(ADXL_345);
        TransmitOne(DATAX0);
        while( ( UCB0IFG & UCSTPIFG ) == 0){} // Polling STOP flag to wait for the message to be done
        UCB0IFG &= ~UCSTPIFG;                   // clear the stop flag

        // Receive process
        Setup_RX(ADXL_345);
        Receive();              // Read ADXL345 acceleration registers
        while( ( UCB0IFG & UCSTPIFG ) == 0){} // Polling STOP flag to wait for the msg to be done
        UCB0IFG &= ~UCSTPIFG;             // clear the stop flag

        x_raw = (float)(((RxBuffer[1]) << 8) | RxBuffer[0]);    // Each axis has two acceleration registers that need to be combined
        y_raw = (float)(((RxBuffer[3]) << 8) | RxBuffer[2]);    // Gives the raw acceleration value
        z_raw = (float)(((RxBuffer[5]) << 8) | RxBuffer[4]);

        x = x_raw / 256;        // Divide raw acceleration value by the scaling factor for +-2g range mode (256) to get acceleration value in g
        y = y_raw / 256;
        z = z_raw / 256;

        Setup_UART();           // Disables I2C interrupts and sets up UART

        // Calculate associated roll and pitch angles
        roll = atan(y / sqrt(pow(x, 2) + pow(z, 2))) * 180 / M_PI;
        pitch = atan(x / sqrt(pow(y, 2) + pow(z, 2))) * 180 / M_PI;

        UCA0_UART_Send(x, y, z, roll, pitch);   // Transmit X, Y, Z acceleration values in g and the associated roll and pitch angles
    }

}
//------------------------------------------------------------
//-- Interrupt Service Routines
#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void)
{
    // Interrupt fires when both the receive and transmit flags are set
    switch(UCB0IV) {    // Identify which flag triggered interrupt

    case 0x16:      // ID 16: RXIFG (Can find this in MSP430FR2355.h)
        *PRxData++ = UCB0RXBUF;    // Read data from RX buffer
        __bic_SR_register_on_exit(CPUOFF);  // Exit LPM0
        break;

    case 0x18:      // ID 18: TXIFG (Can find this in MSP430FR2355.h)
        if (TXByteCtr) {
            TXByteCtr--;
            UCB0TXBUF = MSData[TXByteCtr];  // Value in TX buffer
            __bic_SR_register_on_exit(CPUOFF);  // Exit LPM0
        }
        break;

    default:
        break;
    }
}

#pragma vector = EUSCI_A1_VECTOR
__interrupt void EUSCI_A1_RX_ISR(void)
{
        if(UCA1RXBUF == 's')    // Start command received from XCTU. Wake from low power mode
        {
            P6OUT ^= BIT6;      // Toggle LED2
            __bic_SR_register_on_exit(CPUOFF + SCG0 + SCG1);    // Exit LPM3
        }
        if(UCA1RXBUF == 'p')    // Stop command received from XCTU. Puts MSP430 into low power mode
        {
            P6OUT ^= BIT6;      // Toggle LED2
            __bis_SR_register(CPUOFF + SCG0 + SCG1 + GIE);      // Enter LPM3 with interrupts
        }
}

//---------------------------------------------------------
//-- I2C
void Setup_TX(unsigned char Dev_Addr) {
    UCB0IE &= ~UCRXIE0;         // Disable B0 Rx interrupt (I2C)
    while(UCB0CTLW0 & UCTXSTP); // Ensure STOP condition got sent

    //-- setting up B0 for I2C
    UCB0CTLW0 |= UCSWRST;       // put into SW RST

    UCB0CTLW0 |= UCSSEL_3;      // choose SMCLK
    UCB0BRW = 10;               // set prescalar to 10

    UCB0CTLW0 |= UCMODE_3;      // put into I2C mode
    UCB0CTLW0 |= UCMST;         // put into master mode
    UCB0I2CSA = Dev_Addr;       // set slave address to 0x53 -> ADXL345

    PM5CTL0 &= ~LOCKLPM5;       // Turn on I/O, disable LPM

    UCB0CTLW0 &= ~UCSWRST;      // take B0 out of SW RST

    //-- enable IRQs
    UCB0IE |= UCTXIE0;          // Tx IRQ
    // In a WRITE message, when send START bit, Slave address, WRITE signal, Slave ACKS, this will fire
    // indicating that the transmit buffer is ready
}

void Setup_RX(unsigned char Dev_Addr) {
    UCB0IE &= ~UCTXIE0;         // Disable B0 Tx interrupt (I2C)

    //-- setting up B0 for I2C
    UCB0CTLW0 |= UCSWRST;       // put into SW RST

    UCB0CTLW0 |= UCSSEL_3;      // choose SMCLK
    UCB0BRW = 10;               // set prescalar to 10

    UCB0CTLW0 |= UCMODE_3;      // Put into I2C mode
    UCB0CTLW0 |= UCMST;         // Put into master mode
    UCB0I2CSA = Dev_Addr;       // Set slave address to 0x53 -> ADXL345

    PM5CTL0 &= ~LOCKLPM5;       // Turn on I/O

    UCB0CTLW0 &= ~UCSWRST;      // Take B0 out of SW RST

    UCB0IE |= UCRXIE0;          // Enable Rx IRQ
}

void Transmit(unsigned char Reg_Addr, unsigned char Reg_Data) {
    MSData[1] = Reg_Addr;           // Holds Register address
    MSData[0] = Reg_Data;           // Data written to Reg_Addr
    TXByteCtr = NUM_BYTES_TX;       // # of bytes being transmitted
    UCB0TBCNT = NUM_BYTES_TX;       // # of bytes being transmitted
    UCB0CTLW1 |= UCASTP_2;          // Auto STOP when UCB0TBCNT reached
    while(UCB0CTLW0 & UCTXSTP);     // Ensure STOP condition got sent
    UCB0CTLW0 |= UCTR;              // Put into TX mode
    PM5CTL0 &= ~LOCKLPM5;           // Turn on I/O
    UCB0CTLW0 |= UCTXSTT;           // Generates START condition
    __bis_SR_register(CPUOFF + GIE);    // Enter LPM0 with interrupts
}

void TransmitOne(unsigned char Reg_Addr) {
    MSData[0] = Reg_Addr;       // Holds Register address
    TXByteCtr = 1;              // # of bytes being transmitted
    UCB0TBCNT = 1;              // # of bytes being transmitted
    UCB0CTLW1 |= UCASTP_2;      // auto STOP mode when UCB0TBCNT reached
    while(UCB0CTLW0 & UCTXSTP); // Ensure STOP condition got sent
    UCB0CTLW0 |= UCTR;          // Put into TX mode
    PM5CTL0 &= ~LOCKLPM5;       // Turn on I/O, disable LPM
    UCB0CTLW0 |= UCTXSTT;       // Generates START condition
    __bis_SR_register(CPUOFF + GIE);    // Enter LPM0 with interrupts
}

void Receive(void) {
    PRxData = (unsigned char *)RxBuffer;    // Start of RX buffer
    UCB0TBCNT = NUM_BYTES_RX;       // # of bytes being received
    UCB0CTLW1 |= UCASTP_2;          // Auto STOP when UCB0TBCNT reached
    while(UCB0CTLW0 & UCTXSTP);     // Ensure STOP condition got sent
    UCB0CTLW0 &= ~UCTR;             // Put into RX mode
    PM5CTL0 &= ~LOCKLPM5;           // Turn on I/O
    UCB0CTLW0 |= UCTXSTT;           // Generates START condition
    __bis_SR_register(CPUOFF + GIE);    // Enter LPM0 with interrupts
}

//-------------------------------------------------------------------------------
// UART
//-------------------------------------------------------------------------------
//-- UART Functions to transmit acceleration data to XCTU using UART A0
// These three functions are needed to transmit data to XCTU. The two functions below
// the dotted line must be commented out

void Setup_UART() {         // If using UART A0 to transmit to XCTU using XBee
    UCB0IE &= ~UCRXIE;      // Disable USCI_B0 RX interrupt (I2C)
    UCB0IE &= ~UCTXIE;      // Disable USCI_B0 TX interrupt (I2C)

    //-- Configure UART A0
    UCA0CTLW0 |= UCSWRST;       // Put UART A0 into SW reset

    UCA0CTLW0 |= UCSSEL__ACLK;  // Choose ACLK for UART A0 -> 9600 baud
    UCA0BRW = 3;                // Set prescalar to 3
    UCA0MCTLW = 0x9200;         // Set modulation

    //-- Configure ports
    P1SEL1 &= ~BIT7;            // Set P1.7 to use UART A0 TX function
    P1SEL0 |= BIT7;

    PM5CTL0 &= ~LOCKLPM5;       // Turn on I/O

    UCA0CTLW0 &= ~UCSWRST;      // Take UART A0 out of SW reset
}

void Setup_UART_Rx() {          // For receiving start and stop frame from XCTU. For XBee only.

    //-- setup UART A0
    UCA1CTLW0 |= UCSWRST;        // Put UART A0 into SW reset
    UCA1CTLW0 |= UCSWRST;

    UCA1CTLW0 |= UCSSEL__ACLK;  // Choose ACLK for UART A0 -> 9600 baud
    UCA1BRW = 3;                // Set prescalar to 3
    UCA1MCTLW = 0x9200;         // Set modulation

    //-- configure ports
    P4SEL1 &= ~BIT2;            // Set P4.2 to use UART A0 Rx function
    P4SEL0 |= BIT2;

    P6DIR |= BIT6;              // Set P6.6 to output (LED2)
    P6OUT &= ~BIT6;             // Turn off LED2 initially

    PM5CTL0 &= ~LOCKLPM5;       // Turn on I/O

    UCA1CTLW0 &= ~UCSWRST;      // Take UART A0 out of SW reset

    //-- Setup IRQ A0 RXIFG
    UCA1IE |= UCRXIE;           // Local enable for A0 RXIFG

    __bis_SR_register(CPUOFF + SCG0 + SCG1 + GIE);        // Enter LPM3 with interrupts
}


void UCA0_UART_Send(float x, float y, float z, float roll, float pitch)
{                               // If using UART A0 to transmit data to XCTU
    unsigned char i;

    sprintf(eUSCI_A0_UART.Buffer, "X=%fg Y=%fg Z=%fg Roll=%f° Pitch=%f°\r\n", x, y, z, roll, pitch);

    // CALCULATE #BYTES TO SEND
    eUSCI_A0_UART.Length = strlen(eUSCI_A0_UART.Buffer);

    // ASSIGN POINTER TO THE BUFFER
    eUSCI_A0_UART.pBuf = eUSCI_A0_UART.Buffer;

    while(eUSCI_A0_UART.Length)     // Transmits each ASCII character in eUSCI_A0_UART.Buffer byte by byte
    {
        while (!(UCA0IFG & UCTXIFG));

        UCA0TXBUF = eUSCI_A0_UART.Buffer[i++];
        eUSCI_A0_UART.Length--;

    }
}

//-----------------------------------------------------------------------------------------------------
//-- UART Functions to transmit acceleration data to terminal in CCS using UART A1.
// Use these two functions if printing acceleration data and tilt angle to CCS terminal.
// Comment out the three functions above the dashed line above
/*

void Setup_UART() {       // If using UART A1 to transmit to terminal in CCS
    UCB0IE &= ~UCRXIE; // Disable USCI_B0 RX interrupt (I2C)
    UCB0IE &= ~UCTXIE; // Disable USCI_B0 TX interrupt (I2C)

    //-- setup UART A1
    UCA1CTLW0 |= UCSWRST;        // put UART A1 into SW reset

    UCA1CTLW0 |= UCSSEL__ACLK;  // Choose ACLK for UART A1 -> 9600 baud
    UCA1BRW = 3;                // set prescalar to 3
    UCA1MCTLW = 0x9200;         // set modulation

    //-- configure ports
    P4SEL1 &= ~BIT3;            // Set P4.3 to use UART A1 Tx function
    P4SEL0 |= BIT3;

    PM5CTL0 &= ~LOCKLPM5;       // turn on I/O

    UCA1CTLW0 &= ~UCSWRST;       // take UART A1 out of SW reset
}


void UCA0_UART_Send(float x, float y, float z, float roll, float pitch)           // Sending to CCS terminal using A1 UART
{
    unsigned char i;

    //sprintf(eUSCI_A0_UART.Buffer, "X=%fg Y=%fg Z=%fg Roll=%f° Pitch=%f°\r\n", x, y, z, roll, pitch);
    sprintf(eUSCI_A0_UART.Buffer, "X=%.3fg Y=%.3fg Z=%.3fg Roll=%.3f° Pitch=%.3f°\r\n", x, y, z, roll, pitch);

    // CALCULATE #BYTES TO SEND
    eUSCI_A0_UART.Length = strlen(eUSCI_A0_UART.Buffer);

    // ASSIGN POINTER TO THE BUFFER
    eUSCI_A0_UART.pBuf = eUSCI_A0_UART.Buffer;

    while(eUSCI_A0_UART.Length)     // Transmits each ASCII character in eUSCI_A0_UART.Buffer byte by byte
    {

        while (!(UCA1IFG & UCTXIFG));

        UCA1TXBUF = eUSCI_A0_UART.Buffer[i++];  // NOTE: Using A1 TX buffer NOT A0 TX buffer
        eUSCI_A0_UART.Length--;

    }
}

*/
