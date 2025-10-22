//******************************************************************************
//   MSP430FR243x Demo - eUSCI_B0, I2C Master multiple byte TX/RX
//
//   Description: I2C master communicates to I2C slave sending and receiving
//   3 different messages of different length. I2C master will enter LPM0 mode
//   while waiting for the messages to be sent/receiving using I2C interrupt.
//   ACLK = NA, MCLK = SMCLK = DCO 16MHz.
//
//                                     /|\ /|\
//                   MSP430FR2633      4.7k |
//                 -----------------    |  4.7k
//            /|\ |             P1.3|---+---|-- I2C Clock (UCB0SCL)
//             |  |                 |       |
//             ---|RST          P1.2|-------+-- I2C Data (UCB0SDA)
//                |                 |
//                |                 |
//                |                 |
//                |                 |
//                |                 |
//                |                 |
//
//   Nima Eskandari and Ryan Meredith
//   Texas Instruments Inc.
//   January 2018
//   Built with CCS V7.3
//******************************************************************************

#include <msp430.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define LED_OUT     P1OUT
#define LED_DIR     P1DIR
#define LED0_PIN    BIT0
#define LED1_PIN    BIT1
#define SLAVE_ADDR  0x48
#define MAX_BUFFER_SIZE     20
#define TYPE_0_LENGTH   1

//#define BAUDRATE 9600
//#define CLK_FREQ 16000000UL

//char word[] = "hello";
//volatile uint8_t tx_index = 0;
float temp;// = 33.78;
//unsigned char sBuff[54];
//char sBuff[54] = "temperature:26.62";
//char *sBuff = "temperature is 20.00 degrees";
//float sBuff[] = {56,34.44};
char sBuff[10];// = {00.00,00.00};
char sBuff2[] =  {"Temperature: "};//," Degrees \r\n "};
//char sBuff[] = {00.00,00.00};
//char temp_str[10];
int possition = 0;

//char command1[50], command2[50]; // Added
//char *temp[] = {NULL, command1, command2, NULL}; // Modified
//temp[0]="sum";
//sprintf(temp[1],"%f",x); // remove *
//sprintf(temp[2],"%f",y); // remove *

//char data[] =  {" Temperature: "," Degrees \r\n "};
//char data2[] = { " Degrees \r\n "};
//char *data_pointer = data; // Declare a pointer to the first element of the array
//char data[] = { " Temperature: "," Degrees \r\n "};
void i2c_init();
void init_uart();
//void write_one_byte(char, char);
//uint8_t Read_one_byte(char);
void UART_send_char(char);
void UART_send_string(char*);

void flot(char* p, float x);


//******************************************************************************
// General I2C State Machine ***************************************************
//******************************************************************************

typedef enum I2C_ModeEnum{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;


/* Used to track the state of the software state machine*/
I2C_Mode MasterMode = IDLE_MODE;

/* The Register Address/Command to use*/
uint8_t TransmitRegAddr = 0;

/* ReceiveBuffer: Buffer used to receive data in the ISR
 * RXByteCtr: Number of bytes left to receive
 * ReceiveIndex: The index of the next byte to be received in ReceiveBuffer
 * TransmitBuffer: Buffer used to transmit data in the ISR
 * TXByteCtr: Number of bytes left to transfer
 * TransmitIndex: The index of the next byte to be transmitted in TransmitBuffer
 * */
uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;



/* I2C Write and Read Functions */

/* For slave device with dev_addr, writes the data specified in *reg_data
 *
 * dev_addr: The slave device address.
 *           Example: SLAVE_ADDR
 * reg_addr: The register or command to send to the slave.
 *           Example: CMD_TYPE_0_MASTER
 * *reg_data: The buffer to write
 *           Example: MasterType0
 * count: The length of *reg_data
 *           Example: TYPE_0_LENGTH
 *  */
I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count);

/* For slave device with dev_addr, read the data specified in slaves reg_addr.
 * The received data is available in ReceiveBuffer
 *
 * dev_addr: The slave device address.
 *           Example: SLAVE_ADDR
 * reg_addr: The register or command to send to the slave.
 *           Example: CMD_TYPE_0_SLAVE
 * count: The length of data to read
 *           Example: TYPE_0_LENGTH
 *  */
I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count);
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count);


I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB2I2CSA = dev_addr;
    UCB2IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB2IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB2IE |= UCTXIE;                        // Enable TX interrupt

    UCB2CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(GIE);              // Enter LPM0 w/ interrupts

    return MasterMode;

}


I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;

    //Copy register data to TransmitBuffer
    CopyArray(reg_data, TransmitBuffer, count);

    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB2I2CSA = dev_addr;
    UCB2IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB2IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB2IE |= UCTXIE;                        // Enable TX interrupt

    UCB2CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(GIE);              // Enter LPM0 w/ interrupts

    return MasterMode;
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}


//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************


void initGPIO()
{
    // Configure GPIO
    LED_OUT &= ~(LED0_PIN | LED1_PIN); // P1 setup for LED & reset output
    LED_DIR |= (LED0_PIN | LED1_PIN);

    // I2C pins
    P7SEL0 |= BIT0 | BIT1;
    P7SEL1 &= ~(BIT0 | BIT1);

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
}

void initClockTo16MHz()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    CSCTL0_H = CSKEY_H;                     // Unlock CS registers
    CSCTL1 = DCOFSEL_0;                     // Set DCO to 1MHz

    // Set SMCLK = MCLK = DCO, ACLK = LFXTCLK (VLOCLK if unavailable)
    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;

    // Per Device Errata set divider to 4 before changing frequency to
    // prevent out of spec operation from overshoot transient
    CSCTL3 = DIVA__4 | DIVS__4 | DIVM__4;   // Set all corresponding clk sources to divide by 4 for errata
    CSCTL1 = DCOFSEL_4 | DCORSEL;           // Set DCO to 16MHz

    // Delay by ~10us to let DCO settle. 60 cycles = 20 cycles buffer + (10us / (1/4MHz))
    __delay_cycles(60);
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers to 1 for 16MHz operation
    CSCTL0_H = 0;                           // Lock CS registers
}


void i2c_init()
{
    UCB2CTLW0 = UCSWRST;                      // Enable SW reset
    UCB2CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK | UCSYNC; // I2C master mode, SMCLK
    UCB2BRW = 160;                            // fSCL = SMCLK/160 = ~100kHz
    UCB2I2CSA = SLAVE_ADDR;                   // Slave Address
    UCB2CTLW0 &= ~UCSWRST;                    // Clear SW reset, resume operation
    UCB2IE |= UCNACKIE;
}

void init_uart()
{
    // Configure GPIO
       //     P1OUT &= ~BIT0;                         // Clear P1.0 output latch
       //     P1DIR |= BIT0;                          // For LED on P1.0
            P2SEL0 &= ~(BIT0 | BIT1);
            P2SEL1 |= (BIT0 | BIT1);                // USCI_A3 UART operation

 /*           UCA0CTLW0 |= UCSWRST;
            UCA0CTLW0 |= UCSSEL__SMCLK;     // Using 1 MHZ clock
            UCA0BRW = 6;                    // Baud Rate set to 9600
            UCA0MCTLW = UCOS16 | UCBRF_8 | 0x2000;
*/
            // Configure GPIO
               //     P1OUT &= ~BIT0;                         // Clear P1.0 output latch
               //     P1DIR |= BIT0;                          // For LED on P1.0
//                    P2SEL0 &= ~(BIT0 | BIT1);
//                    P2SEL1 |= (BIT0 | BIT1);                // USCI_A3 UART operation

                    // Baud Rate calculation
                        // 16000000/(16*9600) = 104.1666
                        // Fractional portion = 0.1666
                        // User's Guide Table 30-4: UCBRSx = 0x11
                        // UCBRFx = int ( (104.1666-104)*16) = 2
                    UCA0CTLW0 |= UCSWRST;
                    UCA0CTLW0 |= UCSSEL__SMCLK;     // Using 1 MHZ clock
                    UCA0BRW = 6;                    // Baud Rate set to 9600
                    UCA0MCTLW = UCOS16 | UCBRF_8 | 0x2000;
                    UCA0CTLW0 &= ~UCSWRST;

            // Configure USCI_A3 for UART mode
/*                UCA0CTLW0 |= UCSWRST;
                UCA0CTLW0 |= UCSSEL__SMCLK;                // Set ACLK = 32768 as UCBRCLK
                UCA0BRW = 3;                            // 9600 baud
                UCA0MCTLW |= 0x5300;                    // 32768/9600 - INT(32768/9600)=0.41

*/



}

void UART_send_char(char c)
{
    UCA0TXBUF = 0x00;
    while (!(UCA0IFG & UCTXIFG)); // Wait for TX buffer to be ready
    UCA0TXBUF = c; // Send character
    __delay_cycles(1600);
}

void UART_send_string(char* str)
{
    while (*str)
    {
        UART_send_char(*str++);
    }
}

/*
void UART_send_char(char c)
{
       UCA0TXBUF = 0x00; // write a null byte to clear the transmit buffer
    while (!(UCA0IFG & UCTXIFG)); // Wait for TX buffer to be ready
    UCA0TXBUF = c; // Send character
}

void UART_send_string(char* str)
{
    while (*str)
    {
        UART_send_char(*str++);
    }
}*/

/*void UART_send_string(char *str)
{
    while (*str)
    {
        while (!(UCA0IFG & UCTXIFG)); // Wait for TX buffer to be ready
        UCA0TXBUF = *str++; // Send the current character and increment the pointer
    }
}

void UART_send_char(char c)
{
    while (!(UCA0IFG & UCTXIFG)); // Wait for TX buffer to be ready
    UCA0TXBUF = c; // Send the character
}*/

//******************************************************************************
// Main ************************************************************************
// Send and receive three messages containing the example commands *************
//******************************************************************************

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
   // initClockTo16MHz();
    initGPIO();
    i2c_init();
    init_uart();

    // Disable the GPIO power-on default high-impedance mode to activate
              // previously configured port settings
//              PM5CTL0 &= ~LOCKLPM5;
//              UCA0CTLW0 &= ~UCSWRST;
    while(1)
    {
    //    UCA0IE |= UCTXIE;  // Enable USCI_A0 TX interrupt
    //    __bis_SR_register(GIE);

/*        char buffer[] = "Hello world!"; // declare a character buffer
        int i;

        for (i = 0; i < strlen(buffer); i++)
        {
          UCA0TXBUF = buffer[i]; // transmit each character in the buffer
        }
*/

  //      UART_send_string(sBuff2);
  //      __delay_cycles(16000);
  //      UART_send_string(sBuff2);
  //      __delay_cycles(1600000);

         I2C_Master_WriteReg(SLAVE_ADDR, 1, 0, 1);
         I2C_Master_ReadReg(SLAVE_ADDR, 0, 2);
         CopyArray(ReceiveBuffer, 0, 2);

//         float buffer = 0.0f; // initialize result to zero
//         float *temp = &buffer;
         // perform division using operands in buffer and store result in buffer
         temp = (float)((ReceiveBuffer[0] << 1) | (ReceiveBuffer[1] >> 7)) / 2.0f; // Original code with floating-point division by 2
         __delay_cycles(16000);



         // move result back to its original location
//         float c = *temp;

     //    long i;
        //     for (i=0;i<1000000;i++);

     //    TA1CCTL0 = CCIE;                        // TACCR0 interrupt enabled
     //    TA1CCR0 = 65535;
     //    TA1CTL = TASSEL__ACLK | MC__UP;         // ACLK, up mode
         //TA1CTL = TASSEL__ACLK | MC__CONTINUOUS; // ACLK, continuous mode
     //    __bis_SR_register(GIE);     // Enter LPM3 w/ interrupt
         //TA1CTL &= ~TAIFG;

         // Convert the long integer value to a string
         //char sBuff[54];

   //      sprintf(sBuff,"%f \r\n",temp);

    //     UART_send_string(sBuff2);
         //temp=32.4;
         flot(sBuff,temp);
         __delay_cycles(16000);
         UART_send_string(sBuff);
    //     __delay_cycles(16000);
         UART_send_string("\r\n");
         __delay_cycles(10000);
//scanf("%f",&temp);
//printf("%f",temp);

//         TA0CCTL0 = CCIE;
//         TA0CCR0 = CLK_FREQ / BAUDRATE;
//         TA0CTL = TASSEL__SMCLK | ID__1 | MC__UP;


          //__bis_SR_register(LPM0_bits | GIE);
       //   UCA0IE |= UCRXIE;
 //         UCA0IE |= UCTXIE;  // Enable USCI_A0 TX interrupt
 //         __bis_SR_register(GIE);

         //

   /*        float x;
           char a[20]={};
           char* p=&a;
           printf("Enter the float value.");
           scanf("%f",&x);
           flot(p,x);
           printf("The value=%s",p);
           getchar();*/

         //

    //       ftoa(sBuff,temp);
    //     ftoa(temp,sBuff,2);
    //     snprintf(sBuff, 2, "%f", temp);
     //    sprintf(sBuff,"%f \r\n",ReceiveBuffer[0]);
     //    __delay_cycles(1600000);
//         printf("Temp: %c" , sBuff);
//         __delay_cycles(1600000);

         //gcvt(tempt, 6,sBuff);

         // Convert the long integer value to a string
   //      ftoa(temp, temp_str, 10);
//         UART_send_string("Temperature: \n\r");
//         UART_send_string(sBuff);
//         UART_send_string(" Degreese");
//         __delay_cycles(16000);

   //      __delay_cycles(16000);
   //      UART_send_string("\r\n"); // Newline
    }

//    __bis_SR_register(LPM0_bits + GIE);
//    return 0;
}

void flot(char* p, float x)
{
  int n,i=0,k=0;
  n=(int)x;
  while(n>0)
  {
    x/=10;
    n=(int)x;
    i++;
 }
 *(p+i) = '.';
 x *= 10;
 n = (int)x;
 x = x-n;
 while((n>0)||(i>k))
 {
   if(k == i)
        k++;
   *(p+k)='0'+n;
   x *= 10;
   n = (int)x;
   x = x-n;
   k++;
 }
 /* Null-terminated string */
 *(p+k) = '\0';
}


//******************************************************************************
// I2C Interrupt ***************************************************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B2_VECTOR
__interrupt void USCI_B2_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B2_VECTOR))) USCI_B2_ISR (void)
#else
#error Compiler not supported!
#endif
{
  //Must read from UCB2RXBUF
  uint8_t rx_val = 0;
  switch(__even_in_range(UCB2IV, USCI_I2C_UCBIT9IFG))
  {
    case USCI_NONE:          break;         // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:   break;         // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG:                // Vector 4: NACKIFG
      break;
    case USCI_I2C_UCSTTIFG:  break;         // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:  break;         // Vector 8: STPIFG
    case USCI_I2C_UCRXIFG3:  break;         // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:  break;         // Vector 12: TXIFG3
    case USCI_I2C_UCRXIFG2:  break;         // Vector 14: RXIFG2
    case USCI_I2C_UCTXIFG2:  break;         // Vector 16: TXIFG2
    case USCI_I2C_UCRXIFG1:  break;         // Vector 18: RXIFG1
    case USCI_I2C_UCTXIFG1:  break;         // Vector 20: TXIFG1
    case USCI_I2C_UCRXIFG0:                 // Vector 22: RXIFG0
        rx_val = UCB2RXBUF;
        if (RXByteCtr)
        {
          ReceiveBuffer[ReceiveIndex++] = rx_val;
          RXByteCtr--;
        }

        if (RXByteCtr == 1)
        {
          UCB2CTLW0 |= UCTXSTP;
        }
        else if (RXByteCtr == 0)
        {
          UCB2IE &= ~UCRXIE;
          MasterMode = IDLE_MODE;
          __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
        }
        break;
    case USCI_I2C_UCTXIFG0:                   // Vector 24: TXIFG0
        switch (MasterMode)
        {
          case TX_REG_ADDRESS_MODE:
              UCB2TXBUF = TransmitRegAddr;
              if (RXByteCtr)
                  MasterMode = SWITCH_TO_RX_MODE;   // Need to start receiving now
              else
                  MasterMode = TX_DATA_MODE;        // Continue to transmision with the data in Transmit Buffer
              break;

          case SWITCH_TO_RX_MODE:
              UCB2IE |= UCRXIE;              // Enable RX interrupt
              UCB2IE &= ~UCTXIE;             // Disable TX interrupt
              UCB2CTLW0 &= ~UCTR;            // Switch to receiver
              MasterMode = RX_DATA_MODE;     // State state is to receive data
              UCB2CTLW0 |= UCTXSTT;          // Send repeated start
              if (RXByteCtr == 1)
              {
                  //Must send stop since this is the N-1 byte
                  while((UCB2CTLW0 & UCTXSTT));
                  UCB2CTLW0 |= UCTXSTP;      // Send stop condition
              }
              break;

          case TX_DATA_MODE:
              if (TXByteCtr)
              {
                  UCB2TXBUF = TransmitBuffer[TransmitIndex++];
                  TXByteCtr--;
              }
              else
              {
                  //Done with transmission
                  UCB2CTLW0 |= UCTXSTP;     // Send stop condition
                  MasterMode = IDLE_MODE;
                  UCB2IE &= ~UCTXIE;                       // disable TX interrupt
                  __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
              }
              break;

          default:
              __no_operation();
              break;
        }
        break;
    default: break;
  }
}

// Timer A1 interrupt service routine
/*#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) Timer1_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
 //   P1OUT ^= BIT0;
 //   TA1CCR0 += 10000;                       // Add Offset to TA1CCR0
 //   TA1CTL &= ~TAIFG;

     if (tx_index < sizeof(word))
     {
       UCA0TXBUF = word[tx_index++];
     }
     else
     {
       TA0CTL = MC__STOP; // stop timer
       UCA0IE &= ~UCTXIE; // disable TX interrupt
     }
}


// UART RX Interrupt Service Routine
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
 switch (__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG))
 {
   case USCI_NONE: break;
   case USCI_UART_UCRXIFG:
       UART_send_string("Receive  \r\n");
     break;
   case USCI_UART_UCTXIFG:
       UART_send_string("Transmit  \r\n");
   //    __delay_cycles(1600000);
     break;
   case USCI_UART_UCSTTIFG: break;
   case USCI_UART_UCTXCPTIFG: break;
 }
}*/




