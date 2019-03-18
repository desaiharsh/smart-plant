#include "sensor-interface.h"

volatile unsigned char TXByteBuffer[2];
volatile unsigned char TXByteCtr;
volatile unsigned char RXByteBuffer[2];
volatile unsigned char RXByteCtr;

// Functions to be used by every sensor node.

// MCP9808 Functions

bool mcp9808_init(){
	if (read16(MCP9808_REG_MANUF_ID) != 0x0054)
		return false;
	if (read16(MCP9808_REG_DEVICE_ID) != 0x0400)
		return false;

	write16(MCP9808_REG_CONFIG, 0x0);

	// Add code here to set resolution

	return true;
}

float get_air_temp(){
	uint16_t t = read16(MCP9808_REG_AMBIENT_TEMP);

	float temp = t & 0x0FFF;
	temp /= 16.0;
	if (t & 0x1000)
		temp -= 256;

	return temp;
}

// Si7021 Functions

bool si7021_init(){
  // reset -- was a separate function in the library
  Wire.beginTransmission(_i2caddr);
  Wire.write(SI7021_RESET_CMD);
  Wire.endTransmission();
  delay(50);

  if (_readRegister8(SI7021_READRHT_REG_CMD) != 0x3A)
    return false;

  // Do we need these?
  readSerialNumber();
  _readRevision();

  return true;

}

float get_soil_temp(){
	Wire.beginTransmission(_i2caddr);
	Wire.write(SI7021_MEASTEMP_NOHOLD_CMD);
	uint8_t err = Wire.endTransmission();

	if(err != 0)
		return NAN; //error
    
	uint32_t start = millis(); // start timeout
	while(millis()-start < _TRANSACTION_TIMEOUT) {
		if (Wire.requestFrom(_i2caddr, 3) == 3) {
			uint16_t temp = Wire.read() << 8 | Wire.read();
			uint8_t chxsum = Wire.read();

			float temperature = temp;
			temperature *= 175.72;
			temperature /= 65536;
			temperature -= 46.85;
			return temperature;
			}
		delay(6); // 1/2 typical sample processing time
  	}

	return NAN; // Error timeout
}

float get_humidity(){
	Wire.beginTransmission(_i2caddr);
	
	Wire.write(SI7021_MEASRH_NOHOLD_CMD);
	uint8_t err = Wire.endTransmission();
	if (err != 0)
		return NAN; //error
		
	uint32_t start = millis(); // start timeout
	while(millis()-start < _TRANSACTION_TIMEOUT) {
		if (Wire.requestFrom(_i2caddr, 3) == 3) {
			uint16_t hum = Wire.read() << 8 | Wire.read();
			uint8_t chxsum = Wire.read();
			
			float humidity = hum;
			humidity *= 125;
			humidity /= 65536;
			humidity -= 6;

			return humidity;
			}
		delay(6); // 1/2 typical sample processing time
		}
		return NAN; // Error timeout
}

// ALS-PT19 Functions
float get_light();

// CCS811 Functions
float get_co2();

// Functions to interface sensors

void i2c_init(){

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

bool i2c_send8(uint8_t addr, uint8_t data){

	UCB2I2CSA = addr;// configure slave address

	TXByteCtr = 1;
	TXByteBuffer[0] = data;

	while (UCB2CTLW0 & UCTXSTP);        // Ensure stop condition got sent

        UCB2CTLW0 |= UCTR | UCTXSTT;        // I2C TX, start condition

        __bis_SR_register(LPM0_bits | GIE); // Enter LPM0 w/ interrupts
                                            // Remain in LPM0 until all data
                                            // is TX'd
	return true;
}

bool i2c_send16(uint8_t addr, uint16_t data){

	UCB2I2CSA = addr;// configure slave address

	TXByteCtr = 1;
	TXByteBuffer[0] = data & 0xFF;
	TXByteBuffer[1] = data >> 8;

	while (UCB2CTLW0 & UCTXSTP);        // Ensure stop condition got sent

        UCB2CTLW0 |= UCTR | UCTXSTT;        // I2C TX, start condition

        __bis_SR_register(LPM0_bits | GIE); // Enter LPM0 w/ interrupts
                                            // Remain in LPM0 until all data
                                            // is TX'd
	return true;
}

uint8_t i2c_receive8(uint8_t addr){

	UCB2I2CSA = addr;// configure slave address
	RXByteCtr = 1;

	while (UCB2CTL1 & UCTXSTP);         // Ensure stop condition got sent
        
	UCB2CTL1 |= UCTXSTT;                // I2C start condition

        __bis_SR_register(LPM0_bits | GIE); // Enter LPM0 w/ interrupt

	return RXByteBuffer[0];

}

uint16_t i2c_receive16(uint8_t addr){

	UCB2I2CSA = addr;// configure slave address
	RXByteCtr = 2;

	while (UCB2CTL1 & UCTXSTP);         // Ensure stop condition got sent
        
	UCB2CTL1 |= UCTXSTT;                // I2C start condition

        __bis_SR_register(LPM0_bits | GIE); // Enter LPM0 w/ interrupt

	return ( ((RXByteBuffer[1] << 8) & 0xFF00) | (RXByteBuffer[1]));

}


void __attribute__ ((interrupt(EUSCI_B2_VECTOR))) USCI_B2_ISR (void)
{
    switch(__even_in_range(UCB2IV, USCI_I2C_UCBIT9IFG))
    {
        case USCI_NONE:          break;     // Vector 0: No interrupts
        case USCI_I2C_UCALIFG:   break;     // Vector 2: ALIFG
        case USCI_I2C_UCNACKIFG:            // Vector 4: NACKIFG
            UCB2CTLW0 |= UCTXSTT;           // resend start if NACK
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
            if (--RXByteCtr)                  // Check RX byte counter
            { 
                RXByteBuffer[RXByteCtr] = UCB2RXBUF;             // Get RX data
            }
            else
            {
                RXByteBuffer[RXByteCtr] = UCB2RXBUF;             // Get RX data
                UCB2CTLW0 |= UCTXSTP;       // I2C stop condition
                UCB2IFG &= ~UCRXIFG0;        // Clear USCI_B2 TX int flag
                __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
            }
		break;    
	case USCI_I2C_UCTXIFG0:             // Vector 24: TXIFG0
            if (TXByteCtr)                  // Check TX byte counter
            {
                TXByteCtr--;                // Decrement TX byte counter
                UCB2TXBUF = TXByteBuffer[TXByteCtr];  // Load TX buffer
            }
            else
            {
                UCB2CTLW0 |= UCTXSTP;       // I2C stop condition
                UCB2IFG &= ~UCTXIFG0;        // Clear USCI_B2 TX int flag
                __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
            }
            break;
        case USCI_I2C_UCBCNTIFG: break;     // Vector 26: BCNTIFG
        case USCI_I2C_UCCLTOIFG: break;     // Vector 28: clock low timeout
        case USCI_I2C_UCBIT9IFG: break;     // Vector 30: 9th bit
        default: break;
    }
}
