#include "sensor-interface.h"

volatile unsigned char TXByteBuffer[2];
volatile unsigned char TXByteCtr;
volatile unsigned char RXByteBuffer[2];
volatile unsigned char RXByteCtr;

// Functions to be used by every sensor node.

// MCP9808 Functions

uint8_t mcp9808_init() {
	uint8_t temp;
	//i2c_send8(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_DEVICE_ID);
	//temp = i2c_receive16(MCP9808_I2CADDR_DEFAULT);
#include "sensor-interface.h"

	volatile unsigned char TXByteBuffer[2];
	volatile unsigned char TXByteCtr;
	volatile unsigned char RXByteBuffer[2];
	volatile unsigned char RXByteCtr;

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


	/*
	bool check_mcp9808_init() {
		uint16_t temp;
		//i2c_send8(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_DEVICE_ID);
		//temp = i2c_receive16(MCP9808_I2CADDR_DEFAULT);

		i2c_send8(CCS811_ADDRESS, CCS811_HW_ID);
		temp = i2c_receive16(CCS811_ADDRESS);

		if (temp != CCS811_HW_ID_CODE) {
			return false;
		}
		else return true;
		/*if (read16(MCP9808_REG_MANUF_ID) != 0x0054)
			return false;
		if (read16(MCP9808_REG_DEVICE_ID) != 0x0400)
			return false;

		write16(MCP9808_REG_CONFIG, 0x0);

		// Add code here to set resolution

		//return temp;
	}
	*/
	float si7021_init() {
		uint16_t humdity;
		humdity = read16(SI7021_DEFAULT_ADDRESS, SI7021_MEASRH_NOHOLD_CMD);
		float humidity = humdity;
		humidity *= 125;
		humidity /= 65536;
		humidity -= 6;

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




	/*
	void Adafruit_Si7021::readSerialNumber(void) {
	  Wire.beginTransmission(_i2caddr);

	  i2c_send8((uint8_t)(SI7021_ID1_CMD >> 8));
	  i2c_send8((uint8_t)(SI7021_ID1_CMD & 0xFF));

	  Wire.endTransmission();

	  bool gotData = false;
	  uint32_t start = millis(); // start timeout
	  while(millis()-start < _TRANSACTION_TIMEOUT) {
		if (Wire.requestFrom(_i2caddr, 8) == 8) {
		  gotData = true;
		  break;
		}
		delay(2);
	  }
	  if (!gotData)
		return; // error timeout

	  sernum_a = Wire.read();
	  Wire.read();
	  sernum_a <<= 8;
	  sernum_a |= Wire.read();
	  Wire.read();
	  sernum_a <<= 8;
	  sernum_a |= Wire.read();
	  Wire.read();
	  sernum_a <<= 8;
	  sernum_a |= Wire.read();
	  Wire.read();

	  Wire.beginTransmission(_i2caddr);
	  Wire.write((uint8_t)(SI7021_ID2_CMD >> 8));
	  Wire.write((uint8_t)(SI7021_ID2_CMD & 0xFF));
	  Wire.endTransmission();

	  gotData = false;
	  start = millis(); // start timeout
	  while(millis()-start < _TRANSACTION_TIMEOUT){
		if (Wire.requestFrom(_i2caddr, 8) == 8) {
		  gotData = true;
		  break;
		}
		delay(2);
	  }
	  if (!gotData)
		return; // error timeout

	  sernum_b = Wire.read();
	  Wire.read();
	  sernum_b <<= 8;
	  sernum_b |= Wire.read();
	  Wire.read();
	  sernum_b <<= 8;
	  sernum_b |= Wire.read();
	  Wire.read();
	  sernum_b <<= 8;
	  sernum_b |= Wire.read();
	  Wire.read();

	  switch(sernum_b >> 24) {
		case 0:
		case 0xff:
		  _model = SI_Engineering_Samples;
			break;
		case 0x0D:
		  _model = SI_7013;
		  break;
		case 0x14:
		  _model = SI_7020;
		  break;
		case 0x15:
		  _model = SI_7021;
		  break;
		default:
		  _model = SI_UNKNOWN;
		}
	}*/



	float get_air_temp() {
		uint16_t t = read16(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_AMBIENT_TEMP);

		float temp = t & 0x0FFF;
		temp /= 16.0;
		if (t & 0x1000)
			temp -= 256;

		return temp;

	}



	// Si7021 Functions

	/*bool si7021_init(){
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

	}*/

	/*float get_soil_temp(){
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
	*/

	float get_humidity() {

		/*Wire.beginTransmission(_i2caddr);

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
			*/
	}

	// ALS-PT19 Functions
	float get_light();

	// CCS811 Functions

	uint8_t ccs811_init() {
		uint8_t temp;
		uint8_t temp2;

		UCB2I2CSA = 0xF4;// configure slave address

		TXByteCtr = 0;
		while (UCB2CTLW0 & UCTXSTP);        // Ensure stop condition got sent

		UCB2CTLW0 |= UCTR | UCTXSTT;        // I2C TX, start condition

		__bis_SR_register(LPM0_bits | GIE); // Enter LPM0 w/ interrupts
												// Remain in LPM0 until all data
												// is TX'd


		//i2c_send8(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_DEVICE_ID);
		//temp = i2c_receive16(MCP9808_I2CADDR_DEFAULT);

		//i2c_send8(CCS811_ADDRESS, CCS811_MEAS_MODE);

		//temp = read8(CCS811_ADDRESS, CCS811_MEAS_MODE);

		/*if (read16(MCP9808_REG_MANUF_ID) != 0x0054)
			return false;
		if (read16(MCP9808_REG_DEVICE_ID) != 0x0400)
			return false;

		write16(MCP9808_REG_CONFIG, 0x0);*/

		// Add code here to set resolution
		write8(CCS811_ADDRESS, CCS811_MEAS_MODE, 0x10);

		//i2c_send8(CCS811_ADDRESS, CCS811_MEAS_MODE);


		return temp;
	}

	uint16_t get_co2() {
		//uint16_t eCO2;
		//eCO2 = read16((uint8_t) CCS811_ADDRESS, (uint8_t) CCS811_MEAS_MODE);
		uint8_t co2val;
		co2val = read8(CCS811_ADDRESS, CCS811_ERROR_ID);

		/*uint16_t temp1;
		uint16_t temp2;
		temp1 = co2val & 0x00FF;
		temp2 = co2val & 0xFF00;
		temp1 = temp1 << 8;
		temp2 = temp2 >> 8;
		*/
		return co2val;
		//return temp1 | temp2;
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

		TXByteCtr = 1;
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

		__bis_SR_register(LPM0_bits | GIE); // Enter LPM0 w/ interrupt

	// ( ((RXByteBuffer[1] << 8) & 0xFF00) | (RXByteBuffer[0]));
		return (((RXByteBuffer[1] << 8) & 0xFF00) | (RXByteBuffer[0]));

	}


	void __attribute__((interrupt(EUSCI_B2_VECTOR))) USCI_B2_ISR(void)
	{
		switch (UCB2IV & USCI_I2C_UCBIT9IFG)
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


/*
bool check_mcp9808_init() {
	uint16_t temp;
	//i2c_send8(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_DEVICE_ID);
	//temp = i2c_receive16(MCP9808_I2CADDR_DEFAULT);

	i2c_send8(CCS811_ADDRESS, CCS811_HW_ID);
	temp = i2c_receive16(CCS811_ADDRESS);

	if (temp != CCS811_HW_ID_CODE) {
		return false;
	}
	else return true;
	/*if (read16(MCP9808_REG_MANUF_ID) != 0x0054)
		return false;
	if (read16(MCP9808_REG_DEVICE_ID) != 0x0400)
		return false;

	write16(MCP9808_REG_CONFIG, 0x0);

	// Add code here to set resolution

	//return temp;
}
*/
float si7021_init() {
	uint16_t humdity;
	humdity = read16(SI7021_DEFAULT_ADDRESS, SI7021_MEASRH_NOHOLD_CMD);
	
	uint16_t temp1;
	uint16_t temp2;
	temp1 = humdity & 0x00FF;
	temp2 = humdity & 0xFF00;
	temp1 = temp1 << 8;
	temp2 = temp2 >> 8;

	humdity = temp1 | temp2;

	float humidity = (float)(humdity)/65536;
	//humidity /= 65536;
	humidity *= 125;
	humidity -= 6;


	return humidity;

}

float get_soil_temp() {
	uint16_t temperature;
	temperature = read16(SI7021_DEFAULT_ADDRESS, SI7021_READPREVTEMP_CMD);
	float temp = temperature;
	temp /= 65536;
	temp *= 175.72;
	temp -= 46.85;
	return temp;
}




/*
void Adafruit_Si7021::readSerialNumber(void) {
  Wire.beginTransmission(_i2caddr);

  i2c_send8((uint8_t)(SI7021_ID1_CMD >> 8));
  i2c_send8((uint8_t)(SI7021_ID1_CMD & 0xFF));

  Wire.endTransmission();

  bool gotData = false;
  uint32_t start = millis(); // start timeout
  while(millis()-start < _TRANSACTION_TIMEOUT) {
	if (Wire.requestFrom(_i2caddr, 8) == 8) {
	  gotData = true;
	  break;
	}
	delay(2);
  }
  if (!gotData)
	return; // error timeout

  sernum_a = Wire.read();
  Wire.read();
  sernum_a <<= 8;
  sernum_a |= Wire.read();
  Wire.read();
  sernum_a <<= 8;
  sernum_a |= Wire.read();
  Wire.read();
  sernum_a <<= 8;
  sernum_a |= Wire.read();
  Wire.read();

  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)(SI7021_ID2_CMD >> 8));
  Wire.write((uint8_t)(SI7021_ID2_CMD & 0xFF));
  Wire.endTransmission();

  gotData = false;
  start = millis(); // start timeout
  while(millis()-start < _TRANSACTION_TIMEOUT){
	if (Wire.requestFrom(_i2caddr, 8) == 8) {
	  gotData = true;
	  break;
	}
	delay(2);
  }
  if (!gotData)
	return; // error timeout

  sernum_b = Wire.read();
  Wire.read();
  sernum_b <<= 8;
  sernum_b |= Wire.read();
  Wire.read();
  sernum_b <<= 8;
  sernum_b |= Wire.read();
  Wire.read();
  sernum_b <<= 8;
  sernum_b |= Wire.read();
  Wire.read();

  switch(sernum_b >> 24) {
	case 0:
	case 0xff:
	  _model = SI_Engineering_Samples;
		break;
	case 0x0D:
	  _model = SI_7013;
	  break;
	case 0x14:
	  _model = SI_7020;
	  break;
	case 0x15:
	  _model = SI_7021;
	  break;
	default:
	  _model = SI_UNKNOWN;
	}
}*/



float get_air_temp() {
	uint16_t t = read16(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_AMBIENT_TEMP);

	float temp = t & 0x0FFF;
	temp /= 16.0;
	if (t & 0x1000)
		temp -= 256;

	return temp;

}



// Si7021 Functions

/*bool si7021_init(){
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

}*/

/*float get_soil_temp(){
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
*/

float get_humidity() {

	/*Wire.beginTransmission(_i2caddr);

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
		*/
}

// ALS-PT19 Functions
float get_light();

// CCS811 Functions

uint8_t ccs811_init() {
	uint8_t temp;
	uint8_t temp2;
	//i2c_send8(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_DEVICE_ID);
	//temp = i2c_receive16(MCP9808_I2CADDR_DEFAULT);

	//i2c_send8(CCS811_ADDRESS, CCS811_MEAS_MODE);
	temp = read8(CCS811_ADDRESS, CCS811_MEAS_MODE);

	/*if (read16(MCP9808_REG_MANUF_ID) != 0x0054)
		return false;
	if (read16(MCP9808_REG_DEVICE_ID) != 0x0400)
		return false;

	write16(MCP9808_REG_CONFIG, 0x0);*/

	// Add code here to set resolution
	write8(CCS811_ADDRESS, CCS811_MEAS_MODE, (temp | 0x10));
	//i2c_send8(CCS811_ADDRESS, CCS811_MEAS_MODE);


	return temp;
}

uint16_t get_co2() {
	uint16_t eCO2;
	eCO2 = read16((uint8_t)CCS811_ADDRESS, (uint8_t)CCS811_ALG_RESULT_DATA);
	uint16_t temp1;
	uint16_t temp2;
	temp1 = eCO2 & 0x00FF;
	temp2 = eCO2 & 0xFF00;
	temp1 = temp1 << 8;
	temp2 = temp2 >> 8;

	return temp1 | temp2;
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

	TXByteCtr = 1;
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

	__bis_SR_register(LPM0_bits | GIE); // Enter LPM0 w/ interrupt

	return (((RXByteBuffer[1] << 8) & 0xFF00) | (RXByteBuffer[0]));

}


void __attribute__((interrupt(EUSCI_B2_VECTOR))) USCI_B2_ISR(void)
{
	switch (UCB2IV & USCI_I2C_UCBIT9IFG)
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


