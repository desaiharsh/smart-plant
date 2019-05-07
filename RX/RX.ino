#include <SoftwareSerial.h>
#include <UARTAccess.h>

#define rxPin P6_1
#define txPin P6_0
#define ledPin P1_0
#define RADIO_RESET P4_3

char buff;

void setup()
{
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
  Serial.println("Starting");
}

void loop()
{
  if (Serial.available())
  { // If data comes in from serial monitor, send it out to XBee
    
    //uart_write(Serial.read());
    uint8_t x = Serial.read();
    switch(x){

    case '1':
      Serial.println("Writing +++");
      uart_write('+');
      uart_write('+');
      uart_write('+');
      break;
    case '2':
      Serial.println("Writing ATCH\n");
      uart_write('A');
      uart_write('T');
      uart_write('C');
      uart_write('H');
      uart_write('\r');
      uart_write('\n');
      break;
    default:
      uart_write(x);
      break;
    }
  
  }
  if ( UCA3IFG & UCRXIFG )
  { // If data comes in from XBee, send it out to serial monitor
    //Serial.write(XBee.read());
    buff = uart_read();
    Serial.print(buff);
  }
}
