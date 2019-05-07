#include <SoftwareSerial.h>
#include <UARTAccess.h>

#define rxPin P6_1
#define txPin P6_0
#define ledPin P1_0
#define RADIO_RESET P4_3
#define PRINT_DEBUG
#define NODE_ID 0x02

char buff;

void setup()
{
  delay(4000);
  // Set up both ports at 9600 baud. This value is most important
  // for the XBee. Make sure the baud rate matches the config
  // setting of your XBee.
  Serial.begin(9600);
  //Serial.println("Listening");
    uart_init();
  P3SEL1 |= BIT4;
  P3SEL0 &= BIT4;
  P3DIR |= BIT4;

 pinMode(RADIO_RESET, OUTPUT);
  digitalWrite( RADIO_RESET, LOW );
  delay(1000);
  digitalWrite( RADIO_RESET, HIGH ); 
  Serial.print("Welcome to Node ");
   Serial.println(NODE_ID);
  delay(1000);

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
  
#ifdef PRINT_DEBUG
   Serial.println("Got correct ID. Beginning Sensing.");
#endif
}

void loop()
{
  delay(30000);
  Serial.print("Sending: ");
  uart_write(0xFF);
  Serial.print(0xFF);
  Serial.print(" ");
  uart_write(NODE_ID);
  Serial.print(NODE_ID);
  Serial.println(" ");
}
