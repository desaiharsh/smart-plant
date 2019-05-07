#include <sensor-interface.h>
#define pin P3_1
#define DEBUG_PIN_TOGGLE
#define PRINT_DEBUG
#define RADIO_RST

#define DBG_PIN P8_1
#define SLEEP_PIN P8_0
#define SLEEP_STATUS P4_2
#define RADIO_RESET P4_3

#define NODE_ID 0x01

// =========== GLOBALS DECLARATIONS =====================

uint16_t humidity_si7021;
float temp2;
float temp_mcp;
uint16_t eCO2;
uint16_t light;
float light_reading;
char buff;
uint16_t status_table[3][4];

uint16_t statusPacket, history, reply;

int i, j, flag;
unsigned long time;

// =========== FUNCTION DECLARATIONS =====================
uint8_t write_AT_command(uint8_t *cmd, uint8_t len);
void radio_mode1();
void radio_send_packet(uint8_t data);
void reset_status_table();
void generate_status_packet();  
  
// =========== SETUP =====================

void setup() {
  delay(4000);
#ifdef PRINT_DEBUG
   Serial.begin(9600);
   Serial.print("Welcome to Node ");
   Serial.println(NODE_ID);
#endif

  // ~~~ Initialization Functions
  uart_init();
  i2c_init();
  ccs811_init();

  
#ifdef RADIO_RST
  pinMode(RADIO_RESET, OUTPUT);
  digitalWrite( RADIO_RESET, LOW );
  delay(1000);
  digitalWrite( RADIO_RESET, HIGH );
#endif

  delay(12000);

  radio_mode1();

#ifdef PRINT_DEBUG
  Serial.println("Waiting for Start Message");
#endif
  
  do{
    while ( !(UCA3IFG & UCRXIFG) );
    // Wait for "S" to come in from XBee
    buff = uart_read();
  } while (buff != 'S');

#ifdef PRINT_DEBUG
  Serial.println("Received Start Message. Waiting for correct ID.");
#endif

  
  do{
    while ( !(UCA3IFG & UCRXIFG) );
    // Wait for NODE_ID to come in from XBee
    buff = uart_read();
  } while (buff != (48 + NODE_ID) );
  
  pinMode(pin, INPUT);
  
#ifdef DEBUG_PIN_TOGGLE
  pinMode( DBG_PIN, OUTPUT);
  digitalWrite( DBG_PIN, LOW );
#endif
  
  history = 0;

#ifdef PRINT_DEBUG
   Serial.println("Got correct ID. Beginning Sensing.");
#endif
}

// =========== LOOP =====================

void loop() {

  // ~~~ Initialize Status table
  reset_status_table();
  
  for( i = 0; i < 30 ; i++ ){
    
  // ~~~ Capturing Light Information

#ifdef DEBUG_PIN_TOGGLE
    digitalWrite( DBG_PIN, HIGH);
#endif
    
    light = analogRead(pin);
    light_reading = light*3.3/1023;

#ifdef DEBUG_PIN_TOGGLE
    digitalWrite( DBG_PIN, LOW);
#endif

#ifdef PRINT_DEBUG
    Serial.print("Light: ");
    Serial.print(light);
#endif
    
  // ~~~ Thresholding and setting status bits for light

    if (light > 100 && light < 2000 ) {
      // Status Ok
      status_table[0][0] ++;
    } else if ( light < 100 ) {
      // Generate 01
      status_table[1][0] ++;
    }
    else if ( light > 2000){
      // Generate 10
      status_table[2][0] ++;
      }
      
    delay(250);
    
  // ~~~ Capturing Temperature Information

#ifdef DEBUG_PIN_TOGGLE
    digitalWrite( DBG_PIN, HIGH);
#endif

      temp_mcp = get_air_temp();

#ifdef DEBUG_PIN_TOGGLE
    digitalWrite( DBG_PIN, LOW);
#endif

#ifdef PRINT_DEBUG
      Serial.print(" | MCP temperature: ");
      Serial.print(temp_mcp);
#endif

  // ~~~ Thresholding and setting status bit for temperature
      
      if (temp_mcp > 20.00 && temp_mcp < 40.00) {
        // ~~~ Status Ok 
        status_table[0][1] ++;
      } 
      else if (temp_mcp < 20.00) {
      // ~~~ Status too low 
        status_table[1][1] ++;
      }
      else if (temp_mcp > 40.00) {
      // ~~~ Status too high
        status_table[2][1] ++;
      }
      
    delay(250);

  // ~~~ Capturing Humidity Information

#ifdef DEBUG_PIN_TOGGLE
    digitalWrite( DBG_PIN, HIGH);
#endif

    humidity_si7021 = get_humidity();

#ifdef DEBUG_PIN_TOGGLE
    digitalWrite( DBG_PIN, LOW);
#endif

#ifdef PRINT_DEBUG
    Serial.print(" | si7120 humidity: ");
    Serial.print(humidity_si7021);
#endif

  // ~~~ Thresholding and setting bit for humidity
  
    if (humidity_si7021 > 25.00 && humidity_si7021 < 70.00) {
       // ~~~ Status Ok
       status_table[0][2] ++;
    } 
    else if ( humidity_si7021 < 25.00 ){
        // ~~~ Status too low
        status_table[1][2] ++;
    }
    else if ( humidity_si7021 > 70.00 ){
        // ~~~ Status too high
        status_table[2][2] ++;
    }
    
    delay(250);

  // ~~~ Capturing CO2 Information

#ifdef DEBUG_PIN_TOGGLE
    digitalWrite( DBG_PIN, HIGH);
#endif

    eCO2 = get_co2();

#ifdef DEBUG_PIN_TOGGLE
    digitalWrite( DBG_PIN, LOW);
#endif

#ifdef PRINT_DEBUG
      Serial.print(" | CCS811 CO2 value: ");
      Serial.println(eCO2);
#endif

  // ~~~ Thresholding and setting status bit for CO2
      
      if (eCO2 > 350 && eCO2 < 800) {
        // ~~~ Status Ok 
        status_table[0][3] ++;
      } 
      else if ( eCO2 < 350 ){
        // ~~~ Status too low
        status_table[1][3] ++;
      }
      else if ( eCO2 > 800 ){
        // ~~~ Status too high
        status_table[2][3] ++;
      }
  
   delay(250);
  }

#ifdef PRINT_DEBUG
  for( i = 0; i < 3; i++ ){
    for( j = 0; j < 4; j++ ){
      Serial.print(status_table[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
#endif
  
  generate_status_packet();  

#ifdef PRINT_DEBUG  
  Serial.print("SP: ");
  Serial.println(statusPacket);
#endif
  
  radio_send_packet((uint8_t*)&statusPacket, 2);

  }

// =========== FUNCTION TO WRITE AT COMMAND =====================

uint8_t write_AT_command(uint8_t *cmd, uint8_t len){
      uint8_t i, buff;

      for( i = 0; i < len; i++ ){
        uart_write(cmd[i]);
      }
      
      while ( (UCA3IFG & UCRXIFG) != UCRXIFG);
      buff = uart_read();
      
#ifdef PRINT_DEBUG
      Serial.println(buff);
#endif

      while ( (UCA3IFG & UCRXIFG) != UCRXIFG);
      buff = uart_read();

#ifdef PRINT_DEBUG
    Serial.println(buff);
#endif
      
      return buff;
  
  }

// =========== FUNCTION TO PUT RADIO IN MODE 1 =====================

void radio_mode1(){
  
  // ~~~ Initialize pins
  
  pinMode( SLEEP_STATUS, INPUT);
  
  pinMode( SLEEP_PIN, OUTPUT);
  digitalWrite( SLEEP_PIN, LOW );

  // ~~~ Send AT Command to set SM = 1
  
  reply = write_AT_command( (uint8_t *) "+++", 3);
  delay(100);
  reply = write_AT_command( (uint8_t *) "ATSM1\r\n", 7);

  // ~~~ Wait for AT Command mode to exit

  delay(12000);
}

// =========== FUNCTION TO SEND RADIO PACKET =====================

void radio_send_packet(uint8_t *data, uint8_t len){
  
// ~~~ Exiting Sleep Mode

#ifdef DEBUG_PIN_TOGGLE
    digitalWrite( DBG_PIN, HIGH);
#endif

  digitalWrite( SLEEP_PIN, LOW );
  
#ifdef DEBUG_PIN_TOGGLE
    digitalWrite( DBG_PIN, LOW);
#endif

  delay(1);

#ifdef DEBUG_PIN_TOGGLE
    digitalWrite( DBG_PIN, HIGH);
#endif

// ~~~ Sending Packet

  for( i = 0; i < len; i++ ){
    uart_write(data[i]);
#ifdef PRINT_DEBUG
    Serial.println(data[i]);
#endif
  }

#ifdef DEBUG_PIN_TOGGLE
    digitalWrite( DBG_PIN, LOW);
#endif
  
  delay(250);

  // ~~~ Entering Sleep Mode

#ifdef DEBUG_PIN_TOGGLE
    digitalWrite( DBG_PIN, HIGH);
#endif

  digitalWrite( SLEEP_PIN, HIGH );

#ifdef DEBUG_PIN_TOGGLE
    digitalWrite( DBG_PIN, LOW);
#endif

   delay(1);

  }

// =========== FUNCTION TO RESET STATUS TABLE =====================

void reset_status_table(){
  for( i = 0; i < 3; i++ ){
    for( j = 0; j < 4; j++ ){
      status_table[i][j] = 0;
    }
  }
}

// =========== FUNCTION TO GENERATE STATUS PACKET =====================

void generate_status_packet(){

  uint16_t max_index = 0;

  statusPacket = 0 | (NODE_ID << 8);
  for( j = 0; j < 4; j++ ){
    for( i = 0; i < 3; i++ ){
        if( status_table[i][j] > status_table[max_index][j] ){
            max_index = i;
        }
     }

     statusPacket |= (max_index << 2*j);
     max_index = 0;     
  }
}

