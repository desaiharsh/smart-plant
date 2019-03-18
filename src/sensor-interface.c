#include "sensor-interface.h"

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

void i2c_init(uint8_t addr);
void i2c_send(uint8_t addr);
void i2c_receive(uint8_t addr);


