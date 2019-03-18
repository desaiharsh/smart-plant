#include <msp430.h>

// Struct defining a sensor and it's values.

struct Sensor{

	uint8_t node_id;
	float air_temp;
	float soil_temp;
	float humidity;
	float light;
	float carbon_dioxide;
}Sensor_node;

// Functions to be used by every sensor node.
