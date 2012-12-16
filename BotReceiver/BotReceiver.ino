
//#define DISABLE_MOTORS 
//#define DISABLE_LEDS 
//#define DISABLE_WHEELS
#define DISABLE_ARMS

//#include <SoftwareSerial.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

//
// Hardware configuration
//

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
//RF24 radio(A0,A1);
RF24 radio(9,10);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t addr = 0xFEFEFEFEE1LL;

enum eMode{
  IDLE = 0,
  SYNC = 1,
};

eMode mode = IDLE;
eMode last_mode = IDLE;
const unsigned long idleTime = 5000;
unsigned long lastMsg = idleTime;//start up idling

unsigned char rec_buffer[6]; // our packet: {id}{leds}{motorL}{motorR}{armL}{armR}

// Pin connected to latch pin (ST_CP) of 74HC595
const int latchPin = 4;
// Pin connected to clock pin (SH_CP) of 74HC595
const int clockPin = 2;
// Pin connected to Data in (DS) of 74HC595
const int dataPin = 7;

uint8_t leds = 0xff;  // common anode LEDs, 1 == off
uint8_t motors = 0x00;

void setup(void)
{
  // debugging
  Serial.begin(57600);
  printf_begin();

  printf("\n\rDreamThinkSpeakRobot v0.2\n\r");

  // Setup shift reg
  Serial.println("Setting up shift reg");
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);

  // Setup motors & LEDs 
  Serial.println("Initialising LEDs and Motors");
  registerWrite(leds, motors);

  bothEyesRed();
  registerWrite(leds, motors);
  delay(1000);

  //
  // Setup and configure rf radio
  //
  Serial.println("Initialising RF24");

  initRadio();
  startRadio();

  bothEyesGreen();
  registerWrite(leds, motors);
  delay(1000);

  //
  // Dump the configuration of the rf unit for debugging
  //
  radio.printDetails();

  bothEyesBlue();
  registerWrite(leds, motors);
  delay(1000);

}

uint8_t rcvd_id = 255;
uint8_t rcvd_leds;
int8_t rcvd_wheel_l;
int8_t rcvd_wheel_r;
int8_t rcvd_arm_l;
int8_t rcvd_arm_r;

byte my_id = 1;

unsigned int counter = 0;
unsigned long loop_time = 0;
unsigned long last_tick_millis = 0;

const int tick_duration = 5;

int statusTimer = 0;

void loop(void)
{

  if ( counter % 10000 == 0 ) { 
    Serial.println("tick: counter % 10000 == 0");
    counter = 0;
  }
  if ( counter % 10000 < 5000 and statusTimer == 0 ) {
    bothEyesRed();
  } else if ( counter % 10000 >= 5000 and statusTimer == 0 ) {
    bothEyesBlue();
  }

  // if there is data ready
  if ( radio.available() ) {
    Serial.println("Data received");
    bothEyesGreen();
    statusTimer = 100;
  
    // Dump the payloads until we've gotten everything
    bool done = false;
    while (!done){
      done = radio.read( rec_buffer, 6);
      //update idling time
      lastMsg = millis();
    }

    rcvd_id   = rec_buffer[0]; 
    rcvd_leds = rec_buffer[1];
    rcvd_wheel_l = (int8_t)rec_buffer[2];
    rcvd_wheel_r = (int8_t)rec_buffer[3];
    rcvd_arm_l   = (int8_t)rec_buffer[4];
    rcvd_arm_r   = (int8_t)rec_buffer[5];

    printf("Received:  %d %d %d %d %d %d\n\r", rcvd_id, rcvd_leds, rcvd_wheel_l, rcvd_wheel_r, rcvd_arm_l, rcvd_arm_r );

    mode = SYNC;

  }

  if ( mode == SYNC ) {
     if ( last_mode == IDLE) {
       Serial.println("moving robot");
     }
     move_robot();
  } else {
     if ( last_mode == SYNC ) {
       Serial.println("stopping robot");
     }
     stopRobot();
  }
  
  counter++;

  loop_time = millis();

  last_mode = mode;

  if (millis() - lastMsg > idleTime) {
    if ( last_mode == SYNC ) {
      handleTimeout();
    }
    mode = IDLE;
  }

  if (statusTimer > 0) 
    statusTimer--;

  registerWrite(leds, motors);

}
 
void handleTimeout() {
  Serial.println("timeout -- stopping robot");
  bothEyesCyan();
  statusTimer = 10000;
  radio.printDetails();
}

void bothEyesRed() {
  leds = 0b01110111;
}

void bothEyesGreen() {
  leds = 0b10111011;
}

void bothEyesBlue() {
  leds = 0b11011101;
}

void bothEyesCyan() {
  leds = 0b10011001;
}

void bothEyesPurple() {
  leds = 0b01010101;
}

void bothEyesYellow() {
  leds = 0b10011001;
}

void bothEyesWhite() {
  leds = 0b00010001;
}

void move_robot() {
  //Serial.print("delay: ");
  //Serial.println(millis() - loop_time);
  //return;
  if ( millis() - last_tick_millis > tick_duration ) {
    last_tick_millis = millis();

    if ( rcvd_wheel_l > 0 ) {
       leftWheelForward();
    } else if ( rcvd_wheel_l < 0 ) {
       leftWheelBack();
    } else {
       leftWheelStop();
    }

    if ( rcvd_wheel_r > 0 ) {
       rightWheelForward();
    } else if ( rcvd_wheel_r < 0 ) {
       rightWheelBack();
    } else {
       rightWheelStop();
    }

    if ( rcvd_arm_l > 0 ) {
       leftArmUp();
    } else if ( rcvd_arm_l < 0 ) {
       leftArmDown();
    } else {
       leftArmStop();
    }

    if ( rcvd_arm_r > 0 ) {
       rightArmUp();
    } else if ( rcvd_arm_r < 0 ) {
       rightArmDown();
    } else {
       rightArmStop();
    }

  } 
}

void leftWheelForward() {
  motors &= ~(1 << 5);
  motors |=  (1 << 4);
}

void leftWheelBack() {
  motors &= ~(1 << 4);
  motors |=  (1 << 5);
}

void leftWheelStop() {
  motors &= ~(1 << 5);
  motors &= ~(1 << 4);
}

void leftArmUp() {
  motors &= ~(1 << 7);
  motors |=  (1 << 6);
}

void leftArmDown() {
  motors &= ~(1 << 6);
  motors |=  (1 << 7);
}

void leftArmStop() {
  motors &= ~(1 << 7);
  motors &= ~(1 << 6);
}

void rightWheelForward() {
  motors &= ~(1 << 1);
  motors |= 1;
}

void rightWheelBack() {
  motors &= ~(1);
  motors |=  (1 << 1);
}

void rightWheelStop() {
  motors &= ~(1 << 1);
  motors &= ~(1);
}

void rightArmUp() {
  motors &= ~(1 << 3);
  motors |=  (1 << 2);
}

void rightArmDown() {
  motors &= ~(1 << 2);
  motors |=  (1 << 3);
}

void rightArmStop() {
  motors &= ~(1 << 3);
  motors &= ~(1 << 2);
}

void stopRobot() {
  //return;
  motors = 0;
}

void registerWrite(uint8_t first, uint8_t last) {
  // the bits you want to send. Use an unsigned int,
  // so you can use all 16 bits:
  unsigned int bitsToSend = 0;    

#ifdef DISABLE_MOTORS
  last = 0;
#endif

#ifdef DISABLE_LEDS
  first = 0xff;
#endif

#ifdef DISABLE_ARMS
  last &= 0b00110011;
#endif

#ifdef DISABLE_WHEELS
  last &= 0b11001100;
#endif

  // turn off the output
  digitalWrite(latchPin, LOW);

  // shift the bytes out:
  shiftOut(dataPin, clockPin, MSBFIRST, first);
  shiftOut(dataPin, clockPin, MSBFIRST, last);

  // turn on the output 
  digitalWrite(latchPin, HIGH);
}

void initRadio() {

  radio.begin();
  radio.setChannel(0x69);
  radio.setPayloadSize(6);

}


void startRadio() {

  //setup radio
  radio.openReadingPipe(1,addr);
  radio.startListening();

}

// vim:cin:ai:sts=2 sw=2 ft=cpp
