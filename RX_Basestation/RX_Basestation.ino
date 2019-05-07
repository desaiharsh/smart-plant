#include <SoftwareSerial.h>
#include <UARTAccess.h>
#include <Servo.h> 

#define rxPin P6_1
#define txPin P6_0
#define ledPin P1_0
#define RADIO_RESET P4_3
#define LED_PIN P7_0

uint16_t buff;
int servoPin = P3_2; 

Servo Servo1; 

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

  Servo1.attach(servoPin); 
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(RADIO_RESET, OUTPUT);
  digitalWrite( RADIO_RESET, LOW );
  delay(1000);
  digitalWrite( RADIO_RESET, HIGH ); 
  Serial.println("Starting");
  Servo1.write(180); 
      
}

void loop()
{ 
  if (Serial.available())
  { // If data comes in from serial monitor, send it out to XBee
    
    //uart_write(Serial.read());
    uint8_t x = Serial.read();
    switch(x){

    case 'w':
      // Code for Water Dispensing
      Servo1.write(70); 
      delay(3000); 
   // Make servo go to 180 degrees 
      Servo1.write(180); 
      break;
    case 'l':
      // Code for LED ON
      digitalWrite(LED_PIN, HIGH);
      break;
    case 'k':
      // Code for LED OFF
      digitalWrite(LED_PIN, LOW);
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
    delay(300);
    buff |= uart_read()<<8;
    Serial.println(buff);
    buff = 0;
  }
}
