#include "sensor-interface.h"

volatile unsigned char TXByteBuffer[8];
volatile unsigned char TXByteCtr;
volatile unsigned char RXByteBuffer[8];
volatile unsigned char RXByteCtr;
volatile uint16_t iflag, jflag;

// Functions to be used by every sensor node.

// MCP9808 Functions

uint8_t mcp9808_init() {
	uint8_t temp;
	//i2c_send8(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_DEVICE_ID);
	//temp = i2c_receive16(MCP9808_I2CADDR_DEFAULT);

	i2c_send8(CCS811_ADDRESS, 0x20);
	temp = i2c_receive8(CCS811_ADDRESS);
	/*if (read16(MCP9808_REG_MANUF_ID) != 0x0054)
		return false;
	if (read16(MCP9808_REG_DEVICE_ID) != 0x0400)
		return false;

	write16(MCP9808_REG_CONFIG, 0x0);*/

	// Add code here to set resolution

	return temp;
}


float get_air_temp() {
	uint16_t t = read16(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_AMBIENT_TEMP);

	float temp = t & 0x0FFF;
	temp /= 16.0;
	if (t & 0x1000)
		temp -= 256;

	return temp;

}

// Si7021 Functions

uint16_t get_humidity() {
	uint16_t humdity;
	double humidity;
	iflag = 0;
	i2c_send8(SI7021_DEFAULT_ADDRESS, SI7021_MEASRH_NOHOLD_CMD);
	
	UCB2I2CSA = SI7021_DEFAULT_ADDRESS;// configure slave address
	RXByteCtr = 2;

	UCB2RXBUF = 0;
	while (UCB2CTL1 & UCTXSTP);         // Ensure stop condition got sent

	UCB2CTLW0 &= ~UCTR;	 		       // I2C RX
	UCB2IE |= UCRXIE0;
	UCB2CTL1 |= UCTXSTT;                // I2C start condition
	
	while( (!(UCB2IFG & UCRXIFG0)) && iflag < 20000){ iflag ++; };

	humdity = (((RXByteBuffer[1] << 8) & 0xFF00) | (RXByteBuffer[0]));
	//humdity = read16(SI7021_DEFAULT_ADDRESS, SI7021_MEASRH_HOLD_CMD);
	//return humdity;
	uint32_t temp = humdity;
	temp *= 125;
	humidity = (double)temp/65536;
	humidity -= 6.0;

	return humidity;

}

float get_soil_temp() {
	uint16_t temperature;
	temperature = read16(SI7021_DEFAULT_ADDRESS, SI7021_READPREVTEMP_CMD);
	float temp = temperature;
	temp *= 175.72;
	temp /= 65536;
	temp -= 46.85;
	return temp;
}

// ALS-PT19 Functions
float get_light();

// CCS811 Functions

uint8_t ccs811_init() {
	uint8_t temp;
	
	// ============ SW RESET =====================
	
	UCB2I2CSA = CCS811_ADDRESS;// configure slave address

	TXByteCtr = 5;
	TXByteBuffer[4] = CCS811_SW_RESET;
	TXByteBuffer[3] = 0x11;
	TXByteBuffer[2] = 0xE5;
	TXByteBuffer[1] = 0x72;
	TXByteBuffer[0] = 0x8A;

	while (UCB2CTLW0 & UCTXSTP);        // Ensure stop condition got sent

	UCB2CTLW0 |= UCTR | UCTXSTT;        // I2C TX, start condition

	__bis_SR_register(LPM0_bits | GIE); 
	
	// ============ APP START =====================
		
	__delay_cycles(8000000);
	i2c_send8(CCS811_ADDRESS, CCS811_BOOTLOADER_APP_START);
	__delay_cycles(1600000);

	// ============ SET MEASURE MODE TO  =====================
	
	i2c_send16(CCS811_ADDRESS, 0x0110);
	
	return temp;
}

uint16_t get_co2() {
	uint16_t eCO2;
	eCO2 = read16(CCS811_ADDRESS, CCS811_ALG_RESULT_DATA);
	return eCO2;
}



//UART

void uart_init(void)
{
	// Configure GPIO
	P6SEL1 &= ~(BIT0 | BIT1);
	P6SEL0 |= (BIT0 | BIT1);				// USCI_A3 UART operation

	P6DIR |= BIT0;
	P6DIR &= ~BIT1;

	// Configure USCI_A3 for UART mode
	UCA3CTLW0 = UCSWRST;                    // Put eUSCI in reset
	UCA3CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
	UCA3BRW = 104;                           // 16000000/9600
	UCA3MCTLW |= UCOS16 | UCBRF_2 | 0xD600;
	UCA3CTLW0 &= ~UCSWRST;                  // Initialize eUSCI

}

void uart_write(uint8_t data) {
	UCA3TXBUF = data;
	while (!(UCA3IFG&UCTXIFG));
}

uint8_t uart_read() {
	return UCA3RXBUF;
}


// I2C Functions to interface sensors

uint8_t read8(uint8_t dev_addr, uint8_t reg_addr) {
	uint8_t temp;

	i2c_send8(dev_addr, reg_addr);
	temp = i2c_receive8(dev_addr);

	return temp;

}

uint16_t read16(uint8_t dev_addr, uint8_t reg_addr) {
	uint16_t temp;

	i2c_send8(dev_addr, reg_addr);
	iflag = 0;
	temp = i2c_receive16(dev_addr);

	return temp;

}

void write8(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
	i2c_send8(dev_addr, reg_addr);
	i2c_send8(dev_addr, data);
}

void write16(uint8_t dev_addr, uint8_t reg_addr, uint16_t data) {
	i2c_send8(dev_addr, reg_addr);
	i2c_send16(dev_addr, data);
}


void i2c_init() {

	// Configure GPIO
	P7SEL0 |= BIT0 | BIT1;
	P7SEL1 &= ~(BIT0 | BIT1);
	PM5CTL0 &= ~LOCKLPM5;

	// Configure USCI_B2 for I2C mode
	UCB2CTLW0 = UCSWRST;                    // put eUSCI_B in reset state
	UCB2CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK; // I2C master mode, SMCLK
	UCB2BRW = 128;                          // baudrate = SMCLK(16MHz) / 128 = 125kHz
	UCB2CTLW0 &= ~UCSWRST;                  // clear reset register
	UCB2IE |= UCRXIE0 | UCTXIE0 | UCNACKIE;           // recieve, transmit and NACK interrupt enable

}

bool i2c_send8(uint8_t addr, uint8_t data) {

	UCB2I2CSA = addr;// configure slave address

	TXByteCtr = 2;
	TXByteBuffer[0] = data;

	while (UCB2CTLW0 & UCTXSTP);        // Ensure stop condition got sent

	UCB2CTLW0 |= UCTR | UCTXSTT;        // I2C TX, start condition

	__bis_SR_register(LPM0_bits | GIE); // Enter LPM0 w/ interrupts
										// Remain in LPM0 until all data
										// is TX'd
	return true;
}

bool i2c_send16(uint8_t addr, uint16_t data) {

	UCB2I2CSA = addr;// configure slave address

	TXByteCtr = 3;
	TXByteBuffer[0] = data & 0xFF;
	TXByteBuffer[1] = data >> 8;

	while (UCB2CTLW0 & UCTXSTP);        // Ensure stop condition got sent

	UCB2CTLW0 |= UCTR | UCTXSTT;        // I2C TX, start condition

	__bis_SR_register(LPM0_bits | GIE); // Enter LPM0 w/ interrupts
										// Remain in LPM0 until all data
										// is TX'd
	return true;
}

uint8_t i2c_receive8(uint8_t addr) {

	UCB2I2CSA = addr;// configure slave address
	RXByteCtr = 1;

	while (UCB2CTL1 & UCTXSTP);         // Ensure stop condition got sent

	UCB2CTLW0 &= ~UCTR;	 		       // I2C RX

	UCB2CTL1 |= UCTXSTT;                // I2C start condition

	__bis_SR_register(LPM0_bits | GIE); // Enter LPM0 w/ interrupt

	return RXByteBuffer[0];

}

uint16_t i2c_receive16(uint8_t addr) {
	UCB2I2CSA = addr;// configure slave address
	RXByteCtr = 2;

	UCB2RXBUF = 0;
	while (UCB2CTL1 & UCTXSTP);         // Ensure stop condition got sent

	UCB2CTLW0 &= ~UCTR;	 		       // I2C RX
	
	UCB2CTL1 |= UCTXSTT;                // I2C start condition
	while( UCB2STATW & UCSCLLOW );
	__bis_SR_register(LPM0_bits | GIE); // Enter LPM0 w/ interrupt

// ( ((RXByteBuffer[1] << 8) & 0xFF00) | (RXByteBuffer[0]));
	return (((RXByteBuffer[1] << 8) & 0xFF00) | (RXByteBuffer[0]));

}


void __attribute__((interrupt(EUSCI_B2_VECTOR))) USCI_B2_ISR(void)
{	//iflag = UCB2IV & USCI_I2C_UCBIT9IFG;
	switch (UCB2IV & USCI_I2C_UCBIT9IFG)
	{
	case USCI_NONE:          break;     // Vector 0: No interrupts
	case USCI_I2C_UCALIFG:   break;     // Vector 2: ALIFG
	case USCI_I2C_UCNACKIFG:            // Vector 4: NACKIFG
		UCB2CTL1 |= UCTXSTT;           // resend start if NACK
		break;
	case USCI_I2C_UCSTTIFG:  break;     // Vector 6: STTIFG
	case USCI_I2C_UCSTPIFG:  break;     // Vector 8: STPIFG
	case USCI_I2C_UCRXIFG3:  break;     // Vector 10: RXIFG3
	case USCI_I2C_UCTXIFG3:  break;     // Vector 12: TXIFG3
	case USCI_I2C_UCRXIFG2:  break;     // Vector 14: RXIFG2
	case USCI_I2C_UCTXIFG2:  break;     // Vector 16: TXIFG2
	case USCI_I2C_UCRXIFG1:  break;     // Vector 18: RXIFG1
	case USCI_I2C_UCTXIFG1:  break;     // Vector 20: TXIFG1
	case USCI_I2C_UCRXIFG0: 	    // Vector 22: RXIFG0
		iflag += 50;
		RXByteCtr--;
		if (RXByteCtr > 0)                  // Check RX byte counter
		{
			RXByteBuffer[RXByteCtr] = UCB2RXBUF;             // Get RX data
		}
		else
		{
			UCB2CTLW0 |= UCTXSTP;       // I2C stop condition
			UCB2IFG &= ~UCRXIFG0;        // Clear USCI_B2 RX int flag
			RXByteBuffer[RXByteCtr] = UCB2RXBUF;             // Get RX data
			__bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
		}
		break;
	case USCI_I2C_UCTXIFG0:             // Vector 24: TXIFG0
		iflag = 50;

		if (TXByteCtr > 1)                  // Check TX byte counter
		{
			TXByteCtr--;                // Decrement TX byte counter
			UCB2TXBUF = TXByteBuffer[TXByteCtr-1];  // Load TX buffer
		}
		else if (TXByteCtr == 1)
		{
			UCB2CTLW0 |= UCTXSTP;       // I2C stop condition
			UCB2IFG &= ~UCTXIFG0;        // Clear USCI_B2 TX int flag
			__bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
		}
		break;
	case USCI_I2C_UCBCNTIFG: break;     // Vector 26: BCNTIFG
	case USCI_I2C_UCCLTOIFG: 
		break;     // Vector 28: clock low timeout
	case USCI_I2C_UCBIT9IFG: break;     // Vector 30: 9th bit
	default: break;
	}
}


