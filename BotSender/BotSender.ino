#include <SoftwareSerial.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10

RF24 radio(9,10);

const int ledPin = 13;

// id, leds, left wheel, right wheel, left arm, right arm
signed char msg[] = {
  1, 2, 3, 4, 5, 6};

// leds 0000    0  0  0  0
//          rblue  rred  lblue  lred

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipe = 0xFEFEFEFEE1LL;

//
// Setup
//

void setup(void)
{

  pinMode(ledPin, OUTPUT); 

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
}

//
// Loop
//


void loop(void)
{
  left();
  delay(1000);
  right();
  delay(2000);
  left();
  delay(1000);
  zero();
  waveArms();
  delay(2000);
}

void setLeds(byte eyes){
  msg[1] = eyes;
}

void setMotors(char leftWheel, char rightWheel, char leftArm, char rightArm){
  msg[2] = leftWheel;
  msg[3] = rightWheel;
  msg[4] = leftArm;
  msg[5] = rightArm;
}

void sendBot(){
  bool ok = radio.write(msg, 6);
  if(!ok){
    ok = radio.write(msg, 6);
  }
  if(!ok){
    Serial.println("FAILED to send packet");
  }
}

void left(){
  Serial.println("Turn Left");
  setMotors(-127, 127, 0, 0);
  sendBot();
}

void right(){
  Serial.println("Turn Right");
  setMotors(127, -127, 0, 0);
  sendBot();
}

void waveArms(){
  Serial.println("Left arm");
  setMotors(0, 0, 127, 0);
  sendBot();
  delay(500);
  setMotors(0, 0, -127, 0);
  sendBot();
  delay(500);
  Serial.println("Right arm");
  setMotors(0, 0, 0, 127);
  sendBot();
  delay(500);
  setMotors(0, 0, 0, -127);
  sendBot();
  delay(500);
  zero();
}

void zero(){
  Serial.println("All off");
  setLeds(0);
  setMotors(0, 0, 0, 0);
  sendBot();
}

// vim:ai:cin:sts=2 sw=2 ft=cpp
