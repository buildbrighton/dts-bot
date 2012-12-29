
//#define DISABLE_MOTORS 
//#define DISABLE_LEDS 
//#define DISABLE_WHEELS
#define DISABLE_ARMS
#define ENABLE_RADIO
#define DEBUG_RADIO

enum eArmMode {
  STILL = 0,
  UPDOWN = 1,
  BOOGIE = 2,
  ANGRY = 3,
};

eArmMode leftArmMode = STILL;
eArmMode rightArmMode = STILL;

//#include <SoftwareSerial.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <EEPROM.h>

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
uint8_t myId = 0; // default Id, otherwise address 0 in EEPROM

#define DOWN 1
#define UP   2
#define MID  3

int leftArmState = DOWN;
int rightArmState = DOWN;
int leftArmCounter = 0;
int rightArmCounter = 0;

uint8_t rcvd_id = 255;
uint8_t rcvd_leds;
int8_t rcvd_wheel_l;
int8_t rcvd_wheel_r;
int8_t rcvd_arm_l;
int8_t rcvd_arm_r;

int8_t moveStateWheelL = 0;
int8_t moveStateWheelR = 0;
int8_t moveStateArmL = 0;
int8_t moveStateArmR = 0;

void setup(void)
{
  // debugging
  Serial.begin(57600);
  printf_begin();

  printf("\n\rDreamThinkSpeakRobot v0.3\n\r");

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

#ifdef RESET_EEPROM
  setMyId(255);
#endif

  // go into 'receive an ID mode'
  for ( int i = 0; i<5; i++ ) {
    Serial.println("Awaiting ID...");
    processRadio();
    mode = IDLE;
    if ( rcvd_id > 0 ) {
      Serial.print("Setting ID: ");
      Serial.println(rcvd_id);
      setMyId(rcvd_id);
      //break;
    }
    bothEyesRed();
    registerWrite(leds, 0);
    delay(500);
    bothEyesGreen();
    registerWrite(leds, 0);
    delay(500);
    bothEyesBlue();
    registerWrite(leds, 0);
    delay(500);
  }

  myId = getMyId();

  Serial.print("myID: ");
  Serial.println(myId);

  delay(2000);
  Serial.println("-----------------------------------------");

}

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
  if ( counter % 1000 == 0 ) { 
    Serial.println("tick: counter % 1000 == 0");
  }

  if ( counter % 10000 < 5000 and statusTimer == 0 ) {
    bothEyesRed();
  } else if ( counter % 10000 >= 5000 and statusTimer == 0 ) {
    bothEyesBlue();
  }

  processRadio();

  processArms();

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
  if ( leftArmCounter >= 10000 ) 
    leftArmCounter = 0;
  else 
    leftArmCounter++;

  if ( rightArmCounter >= 10000 ) 
    rightArmCounter = 0;
  else 
    rightArmCounter++;

  loop_time = millis();
  last_mode = mode;
  if (statusTimer > 0) 
    statusTimer--;

  if (millis() - lastMsg > idleTime) {
    if ( last_mode == SYNC ) {
      handleTimeout();
    }
    mode = IDLE;
  }

  registerWrite(leds, motors);

  delay(1);

}
 
void handleTimeout() {
  Serial.println("timeout -- stopping robot");
  bothEyesCyan();
  statusTimer = 10000;
  radio.printDetails();
  initRadio();
  startRadio();
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

    if ( moveStateWheelL > 0 ) {
       leftWheelForward();
    } else if ( moveStateWheelL < 0 ) {
       leftWheelBack();
    } else {
       leftWheelStop();
    }

    if ( moveStateWheelR > 0 ) {
       rightWheelForward();
    } else if ( moveStateWheelR < 0 ) {
       rightWheelBack();
    } else {
       rightWheelStop();
    }

    if ( moveStateArmL > 0 ) {
       leftArmUp();
    } else if ( moveStateArmL < 0 ) {
       leftArmDown();
    } else {
       leftArmStop();
    }

    if ( moveStateArmR > 0 ) {
       rightArmUp();
    } else if ( moveStateArmR < 0 ) {
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

bool leftArmStopperButton() {
  return 1;
}

bool rightArmStopperButton() {
  return 1;
}

bool pickedUpButton() {
  return 1;
}

void leftArmReset() {
  return;
}

void rightArmReset() {
  return;
}

void leftArmUp() {
  //Serial.println("leftArmUp");
  motors &= ~(1 << 7);
  motors |=  (1 << 6);
}

void leftArmDown() {
  //Serial.println("leftArmDown");
  motors &= ~(1 << 6);
  motors |=  (1 << 7);
}

void leftArmStop() {
  motors &= ~(1 << 7);
  motors &= ~(1 << 6);
}

void rightArmUp() {
  //Serial.println("rightArmUp");
  motors &= ~(1 << 3);
  motors |=  (1 << 2);
}

void rightArmDown() {
  //Serial.println("rightArmDown");
  motors &= ~(1 << 2);
  motors |=  (1 << 3);
}

void rightArmStop() {
  motors &= ~(1 << 3);
  motors &= ~(1 << 2);
}

void stopRobot() {
  return;
  motors = 0;
}

void registerWrite(uint8_t leds, uint8_t motors) {
  // the bits you want to send. Use an unsigned int,
  // so you can use all 16 bits:
  unsigned int bitsToSend = 0;    

  if ( counter % 500 == 0 ) {
    Serial.print("leds: ");
    Serial.print(leds, BIN);
    Serial.print(", motors: ");
    Serial.print(motors, BIN);
    Serial.println();
  }

#ifdef DISABLE_MOTORS
  motors = 0;
#endif

#ifdef DISABLE_LEDS
  leds = 0xff;
#endif

#ifdef DISABLE_ARMS
  motors &= 0b00110011;
#endif

#ifdef DISABLE_WHEELS
  motors &= 0b11001100;
#endif

  // turn off the output
  digitalWrite(latchPin, LOW);

  // shift the bytes out:
  shiftOut(dataPin, clockPin, MSBFIRST, leds);
  shiftOut(dataPin, clockPin, MSBFIRST, motors);

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

void processRadio() {
#ifdef ENABLE_RADIO
  // if there is data ready
  if ( radio.available() ) {
#ifdef DEBUG_RADIO
    Serial.println("Data received");
#endif
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

#ifdef DEBUG_RADIO
    printf("Received:  %d %d %d %d %d %d\n\r", rcvd_id, rcvd_leds, rcvd_wheel_l, rcvd_wheel_r, rcvd_arm_l, rcvd_arm_r );
#endif

    if ( rcvd_id == myId ) {
      mode = SYNC;
      moveStateWheelL = rcvd_wheel_l;
      moveStateWheelR = rcvd_wheel_r;
      moveStateArmL   = rcvd_arm_l;
      moveStateArmR =   rcvd_arm_r;
    }

  }
#endif
}

void setMyId(uint8_t id) {
  Serial.print("Setting myID in EEPROM to: ");
  Serial.println(id, DEC);
  EEPROM.write(0, id);
}

uint8_t getMyId() {
  uint8_t id = EEPROM.read(0);
  if ( id == 255 ) {
    return 0;
  } else {
    return id;
  }
}

void processArms() {
  
  // left arm
  if ( leftArmStopperButton() == 0 ) {
    leftArmReset();
  } else if (leftArmMode == STILL ) {
    leftArmStop();
  } else if (leftArmMode == UPDOWN ) {
    if ( leftArmCounter % 1000 == 0 ) {
      leftArmStop();
    } else if ( leftArmCounter % 1000 == 333 ) {
      leftArmUp();
    } else if ( leftArmCounter % 1000 == 666 ) {
      leftArmDown();
    }
  } else if (leftArmMode == BOOGIE ) {
    if ( leftArmCounter % 500 == 250 ) {
      leftArmUp();
    } else if ( leftArmCounter % 500 == 250 ) {
      leftArmDown();
    }
  }

  // right arm
  if ( rightArmStopperButton() == 0 ) {
    rightArmReset();
  } else if (rightArmMode == STILL ) {
    rightArmStop();
  } else if (rightArmMode == UPDOWN ) {
    if ( rightArmCounter % 1000 == 0 ) {
      rightArmStop();
    } else if ( rightArmCounter % 1000 == 333 ) {
      rightArmUp();
    } else if ( rightArmCounter % 1000 == 666 ) {
      rightArmDown();
    }
  } else if (rightArmMode == BOOGIE ) {
    if ( rightArmCounter % 500 == 250 ) {
      rightArmUp();
    } else if ( rightArmCounter % 500 == 0 ) {
      rightArmDown();
    }
  }

}

// vim:cin:ai:sts=2 sw=2 ft=cpp
