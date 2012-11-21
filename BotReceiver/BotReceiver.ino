
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

int wheel_left[]  = {11, 12};
int wheel_right[] = {13, 14};

int arm_left[]  = {16, 17};
int arm_right[] = {18, 19};

enum eMode{
  IDLE = 0,
  SYNC = 1,
};

eMode mode = IDLE;
eMode last_mode = IDLE;
const unsigned long idleTime = 5000;
unsigned long lastMsg = idleTime;//start up idling

unsigned char rec_buffer[6]; // our packet: {id}{leds}{motorL}{motorR}{armL}{armR}

void setup(void)
{
  // Setup motors
  for(int i = 0; i < 2; i++){
    pinMode(wheel_left[i], OUTPUT);
    pinMode(wheel_right[i], OUTPUT);
    pinMode(arm_left[i], OUTPUT);
    pinMode(arm_right[i], OUTPUT);
  }

  // debugging
  Serial.begin(57600);
  printf_begin();

  printf("\n\rDreamThinkSpeakRobot v0.1\n\r");

  //
  // Setup and configure rf radio
  //

  radio.begin();
  radio.setChannel(0x69);
  radio.setPayloadSize(6);
  //radio.disableCRC();

  //
  // Open pipes to other nodes for communication
  //

  //setup radio
  radio.openReadingPipe(1,addr);
  radio.startListening();

  //
  // Dump the configuration of the rf unit for debugging
  //
  radio.printDetails();

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

void loop(void)
{

  // if there is data ready
  if ( radio.available() ) {
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
     if ( last_mode == IDLE)
       Serial.println("moving robot");
     move_robot();
  } else {
     if ( last_mode == SYNC )
       Serial.println("stopping robot");
     stop_robot();
  }
  
  counter++;

  loop_time = millis();

  last_mode = mode;

  if (millis() - lastMsg > idleTime) {
    if ( last_mode == SYNC )
      Serial.println("timeout -- stopping robot");
    mode = IDLE;
  }

}

void move_robot() {
  //Serial.print("delay: ");
  //Serial.println(millis() - loop_time);
  if ( millis() - last_tick_millis > tick_duration ) {
    //Serial.println("tick");
    last_tick_millis = millis();

    if ( rcvd_wheel_l > 0 ) {
       Serial.print("left wheel forward");
       digitalWrite(wheel_left[0], HIGH);
       digitalWrite(wheel_left[1], LOW);
    } else if ( rcvd_wheel_l < 0 ) {
       Serial.print("left wheel back");
       digitalWrite(wheel_left[0], LOW);
       digitalWrite(wheel_left[1], HIGH);
    } else {
       Serial.print("left wheel stop");
       digitalWrite(wheel_left[0], LOW);
       digitalWrite(wheel_left[1], LOW);
    }

    if ( rcvd_wheel_r > 0 ) {
       Serial.print("right wheel forward");
       digitalWrite(wheel_right[0], HIGH);
       digitalWrite(wheel_right[1], LOW);
    } else if ( rcvd_wheel_r < 0 ) {
       Serial.print("right wheel back");
       digitalWrite(wheel_right[0], LOW);
       digitalWrite(wheel_right[1], HIGH);
    } else {
       Serial.print("right wheel stop");
       digitalWrite(wheel_right[0], LOW);
       digitalWrite(wheel_right[1], LOW);
    }

    if ( rcvd_arm_l > 0 ) {
       Serial.print("left arm up");
       digitalWrite(arm_left[0], HIGH);
       digitalWrite(arm_left[1], LOW);
    } else if ( rcvd_arm_l < 0 ) {
       Serial.print("left arm down");
       digitalWrite(arm_left[0], LOW);
       digitalWrite(arm_left[1], HIGH);
    } else {
       Serial.print("left arm stop");
       digitalWrite(arm_left[0], LOW);
       digitalWrite(arm_left[1], LOW);
    }

    if ( rcvd_arm_r > 0 ) {
       Serial.print("right arm up");
       digitalWrite(arm_right[0], HIGH);
       digitalWrite(arm_right[1], LOW);
    } else if ( rcvd_arm_r < 0 ) {
       Serial.print("right arm down");
       digitalWrite(arm_right[0], LOW);
       digitalWrite(arm_right[1], HIGH);
    } else {
       Serial.print("right arm stop");
       digitalWrite(arm_right[0], LOW);
       digitalWrite(arm_right[1], LOW);
    }

  } 
}

void stop_robot() {
  digitalWrite(wheel_left[0], LOW); 
  digitalWrite(wheel_left[1], LOW); 
  digitalWrite(wheel_right[0], LOW); 
  digitalWrite(wheel_right[1], LOW);
  digitalWrite(arm_left[0], LOW); 
  digitalWrite(arm_left[1], LOW); 
  digitalWrite(arm_right[0], LOW); 
  digitalWrite(arm_right[1], LOW);
}

// vim:cin:ai:sts=2 sw=2 ft=cpp
