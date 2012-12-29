#include <SoftwareSerial.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

//#define CHECK_RADIO_RECEIPT


// Set up nRF24L01 radio on SPI bus plus pins 9 & 10

RF24 radio(9,10);

// id, leds, left wheel, right wheel, left arm, right arm
signed char msg[] = { 1, 2, 3, 4, 5, 6};

#define NUM_ROBOTS 2
signed char leds[NUM_ROBOTS];
signed char wheelL[NUM_ROBOTS];
signed char wheelR[NUM_ROBOTS];
signed char armL[NUM_ROBOTS];
signed char armR[NUM_ROBOTS];
long robotCounter[NUM_ROBOTS];
long robotEnable[NUM_ROBOTS];

// leds 0000    0  0  0  0
//          rblue  rred  lblue  lred

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipe = 0xFEFEFEFEE1LL;

//
// Setup
//

const int redPin = 6;
const int greenPin = 5;
const int bluePin = 3;
const int buttonPin = 7;

int counter = 0;
int statusTimer = 0;

void setup(void)
{

  pinMode(redPin, OUTPUT); 
  pinMode(greenPin, OUTPUT); 
  pinMode(bluePin, OUTPUT); 
  digitalWrite(redPin, HIGH);  // common-anode
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin, HIGH);

  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, HIGH);  // set pull-up

  //
  // Print preamble
  //

  Serial.begin(57600);
  printf_begin();

  printf("\n\rRF24/Bot send/\n\r");

  //
  // Setup and configure rf radio
  //

  radio.begin();
#ifndef CHECK_RADIO_RECEIPT
  radio.disableCRC();
#endif
  radio.setChannel(0x69);
  radio.setPayloadSize(6);

  //
  // Open pipes to other nodes for communication
  //

  //Debug over the serial
  Serial.begin(57600);

  //setup radio
  radio.openWritingPipe(pipe);

  //
  // Dump the configuration of the rf unit for debugging
  //
  radio.printDetails();


  // initialise arrays
  for ( int i; i<NUM_ROBOTS; i++ ) {
     robotEnable[i] = 0;
     robotCounter[i] = 0;
     leds[i] = 0;
     wheelL[i] = 0;
     wheelR[i] = 0;
     armL[i] = 0;
     armR[i] = 0;
  }

  // default robot always enabled
  robotEnable[0] = 1;

}

//
// Loop
//



void loop(void)
{
  if ( counter % 10000 == 0 ) {
#ifdef DEBUG_RADIO
    radio.printDetails();
#endif
    Serial.println("Counter % 10000 == 0");
    counter = 0;
  }

  if ( counter % 1000 == 0 ) {
    Serial.println("tick...");
  }

  if ( counter % 10000 < 5000 && statusTimer == 0 ) {
    statusBlue();
  } else if ( counter % 10000 >= 5000 && statusTimer == 0 ) {
    statusGreen();
  }

  if ( buttonPressed() ) {
    robotEnable[1] = 1;
  }

  if ( counter % 10000 == 1000 ) 
    left(0);

  if ( counter % 10000 == 2000 ) 
    right(0);
    
  if ( counter % 10000 == 3000 ) 
    zero(0);

  if ( robotEnable[1] == 1 ) {

    if ( robotCounter[1] == 0 ) 
      forward(1);

    if ( robotCounter[1] == 1000 ) 
      right(1);

    if ( robotCounter[1] == 1500 ) 
      forward(1);

    if ( robotCounter[1] == 2500 ) 
      right(1);

    if ( robotCounter[1] == 3500 ) 
      forward(1);

    if ( robotCounter[1] == 4500 )
      left(1);

    if ( robotCounter[1] == 5000 )
      forward(1);

    if ( robotCounter[1] == 6000 )
      zero(1);

    if ( robotCounter[1] > 6000 ) {
      robotEnable[1] = 0;
      robotCounter[1] = 0;
    } else {
      robotCounter[1]++;
    }

  } 

  if (statusTimer > 0)
      statusTimer--;

  counter++;

  // repeatedly send commands to robots
  if ( counter % 10 == 0 ) {
    for ( uint8_t i; i<NUM_ROBOTS; i++ ) {
      if ( robotEnable[i] ) {
        sendBot(i);
      }
    }
  }

  delay(1);

}

void setLeds(uint8_t id, byte eyes){
  leds[id] = eyes;
}

void setWheels(uint8_t id, char left, char right){
  wheelL[id] = left;
  wheelR[id] = right;
}

void setArms(uint8_t id, char left, char right){
  armL[id] = left;
  armR[id] = right;
}

void statusRed() {
  digitalWrite(redPin,   LOW);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin,  HIGH);
}

void statusGreen() {
  digitalWrite(redPin,   HIGH);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin,  HIGH);
}

void statusBlue() {
  digitalWrite(redPin,   HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin,  LOW);
}

void sendBot(uint8_t id){

  msg[0] = id;
  msg[1] = leds[id];
  msg[3] = wheelL[id];
  msg[4] = wheelR[id];
  msg[5] = armL[id];
  msg[6] = armR[id];

  bool ok = radio.write(msg, 6);
#ifdef CHECK_RADIO_RECEIPT
  if(!ok){
    ok = radio.write(msg, 6);
  }
  if(!ok){
    Serial.println("FAILED to send packet");
    statusRed();
    statusTimer = 100;
  }
#endif
}

void left(uint8_t id){
  Serial.print("ID ");
  Serial.print(id, DEC);
  Serial.println(" - Rotate Left");
  setWheels(id, -127, 127);
}

void forward(uint8_t id) {
  Serial.print("ID ");
  Serial.print(id, DEC);
  Serial.println(" - Go forward");
  setWheels(id, 127, 127);
}

void back(uint8_t id) {
  Serial.print("ID ");
  Serial.print(id, DEC);
  Serial.println(" - Go back");
  setWheels(id, -127, -127);
}

void right(uint8_t id){
  Serial.print("ID ");
  Serial.print(id, DEC);
  Serial.println(" - Rotate right");
  setWheels(id, 127, -127);
}

void zero(uint8_t id){
  Serial.println("All off");
  setLeds(id, 0);
  setWheels(id, 0, 0);
  setArms(id, 0, 0);
}

bool buttonPressed() {

  if ( digitalRead(buttonPin) == HIGH ) {
    return 0;
  } else {
    return 1;
  }

}

// vim:ai:cin:sts=2 sw=2 ft=cpp
