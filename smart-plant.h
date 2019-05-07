/*
  ************************************************************************
  * 	sensor-interface.h
  *
  * 	Struct definitions and function declarations relevant to sensor nodes
  *
  *     Contribution: Harsh Desai, Varsha Prasad Narsing, Jaidev Singh Chadha
  *
  *     Sensors Used:
  *     1. MCP9808 Temperature Sensor : https://learn.adafruit.com/adafruit-mcp9808-precision-i2c-temperature-sensor-guide
  *     2. Si7021 Humidity & Temperature Sensor : https://learn.adafruit.com/adafruit-si7021-temperature-plus-humidity-sensor
  *     3. ALS-PT19 Analog Light Level Sensor : https://www.adafruit.com/product/2748
  *     4. CCS811 CO2 Sensor : https://learn.adafruit.com/adafruit-ccs811-air-quality-sensor
  *
  ************************************************************************
*/

#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>

// ----------------------------- I2C Addresses ---------------------------

// MCP9808 Temperature Sensor

#define MCP9808_I2CADDR_DEFAULT 0x18 ///< I2C address
#define MCP9808_REG_CONFIG 0x01      ///< MCP9808 config register

#define MCP9808_REG_CONFIG_SHUTDOWN 0x0100   ///< shutdown config
#define MCP9808_REG_CONFIG_CRITLOCKED 0x0080 ///< critical trip lock
#define MCP9808_REG_CONFIG_WINLOCKED 0x0040  ///< alarm window lock
#define MCP9808_REG_CONFIG_INTCLR 0x0020     ///< interrupt clear
#define MCP9808_REG_CONFIG_ALERTSTAT 0x0010  ///< alert output status
#define MCP9808_REG_CONFIG_ALERTCTRL 0x0008  ///< alert output control
#define MCP9808_REG_CONFIG_ALERTSEL 0x0004   ///< alert output select
#define MCP9808_REG_CONFIG_ALERTPOL 0x0002   ///< alert output polarity
#define MCP9808_REG_CONFIG_ALERTMODE 0x0001  ///< alert output mode

#define MCP9808_REG_UPPER_TEMP 0x02   ///< upper alert boundary
#define MCP9808_REG_LOWER_TEMP 0x03   ///< lower alert boundery
#define MCP9808_REG_CRIT_TEMP 0x04    ///< critical temperature
#define MCP9808_REG_AMBIENT_TEMP 0x05 ///< ambient temperature
#define MCP9808_REG_MANUF_ID 0x06     ///< manufacture ID
#define MCP9808_REG_DEVICE_ID 0x07    ///< device ID
#define MCP9808_REG_RESOLUTION 0x08   ///< resolutin


// Si7021 Humidity + Temperature Sensor

#define SI7021_DEFAULT_ADDRESS	0x40

#define SI7021_MEASRH_HOLD_CMD           0xE5 // Measure Relative Humidity, Hold Master Mode
#define SI7021_MEASRH_NOHOLD_CMD         0xF5 // Measure Relative Humidity, No Hold Master Mode
#define SI7021_MEASTEMP_HOLD_CMD         0xE3 // Measure Temperature, Hold Master Mode
#define SI7021_MEASTEMP_NOHOLD_CMD       0xF3 // Measure Temperature, No Hold Master Mode
#define SI7021_READPREVTEMP_CMD          0xE0 // Read Temperature Value from Previous RH Measurement
#define SI7021_RESET_CMD                 0xFE
#define SI7021_WRITERHT_REG_CMD          0xE6 // Write RH/T User Register 1
#define SI7021_READRHT_REG_CMD           0xE7 // Read RH/T User Register 1
#define SI7021_WRITEHEATER_REG_CMD       0x51 // Write Heater Control Register
#define SI7021_READHEATER_REG_CMD        0x11 // Read Heater Control Register
#define SI7021_ID1_CMD                   0xFA0F // Read Electronic ID 1st Byte
#define SI7021_ID2_CMD                   0xFCC9 // Read Electronic ID 2nd Byte
#define SI7021_FIRMVERS_CMD              0x84B8 // Read Firmware Revision

#define SI7021_REV_1					0xff
#define SI7021_REV_2					0x20

// CCS811 Carbon Dioxide Level Sensor

/*=========================================================================
	I2C ADDRESS/BITS
	-----------------------------------------------------------------------*/
#define CCS811_ADDRESS                (0x5A)
	/*=========================================================================*/

	/*=========================================================================
		REGISTERS
		-----------------------------------------------------------------------*/

#define CCS811_STATUS 0x00
#define CCS811_MEAS_MODE 0x01

#define CCS811_ALG_RESULT_DATA 0x02

#define CCS811_RAW_DATA 0x03
#define CCS811_ENV_DATA 0x05
#define CCS811_NTC 0x06
#define CCS811_THRESHOLDS 0x10
#define CCS811_BASELINE 0x11
#define CCS811_HW_ID 0x20
#define CCS811_HW_VERSION 0x21
#define CCS811_FW_BOOT_VERSION 0x23
#define CCS811_FW_APP_VERSION 0x24
#define CCS811_ERROR_ID 0xE0
#define CCS811_SW_RESET 0xFF


		//bootloader registers

#define CCS811_BOOTLOADER_APP_ERASE 0xF1
#define CCS811_BOOTLOADER_APP_DATA 0xF2
#define CCS811_BOOTLOADER_APP_VERIFY 0xF3
#define CCS811_BOOTLOADER_APP_START 0xF4



#define CCS811_DRIVE_MODE_IDLE 0x00
#define CCS811_DRIVE_MODE_1SEC 0x01
#define CCS811_DRIVE_MODE_10SEC 0x02
#define CCS811_DRIVE_MODE_60SEC 0x03
#define CCS811_DRIVE_MODE_250MS 0x04

/*=========================================================================*/

#define CCS811_HW_ID_CODE			0x81

#define CCS811_REF_RESISTOR			100000


// Struct defining a sensor and it's values.

/*struct Sensor{

	uint8_t node_id;
	float air_temp;
	float soil_temp;
	float humidity;
	float light;
	float carbon_dioxide;
}Sensor_node;
*/
// Functions to be used by every sensor node.

// MCP9808 Functions

uint8_t mcp9808_init();
bool check_mcp9808_init();
float get_air_temp();
// void shutdown_wake(boolean sw);
// void shutdown();
// void wake();


// Si7021 Functions

float si7021_init();
float get_soil_temp();
uint16_t get_humidity();

// ALS-PT19 Functions

float get_light();

// CCS811 Functions
uint8_t ccs811_init();

uint16_t get_co2();

//UART
void uart_init();
void uart_write(uint8_t data);
uint8_t uart_read();

// Functions to interface sensors

uint8_t read8(uint8_t dev_addr, uint8_t reg_addr);
uint16_t read16(uint8_t dev_addr, uint8_t reg_addr);
void write8(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
void write16(uint8_t dev_addr, uint8_t reg_addr, uint16_t data);
void i2c_init();
bool i2c_send8(uint8_t addr, uint8_t data);
bool i2c_send16(uint8_t addr, uint16_t data);
uint8_t i2c_receive8(uint8_t addr);
uint16_t i2c_receive16(uint8_t addr);

