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

  setLeds(true, false, false, false);
  Serial.println(msg[1]);  
  setLeds(false, false, false, true);
  Serial.println(msg[1]);
}

//
// Loop
//

signed char thing = -50;

void loop(void)
{
  unsigned char msg[] = {
    1, 2, 3, 4, -5, thing  };


  thing++;

  delay(10);

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
  setMotors(-127, 127, 0, 0);
}

void right(){
  setMotors(127, -127, 0, 0);
}

void waveArm(){
  setMotors(0, 0, 127, 0);
  delay(500);
  setMotors(0, 0, -127, 0);
  delay(500);
  zero();
}

void zero(){
  setLeds(0);
  setMotors(0, 0, 0, 0);
}

// vim:ai:cin:sts=2 sw=2 ft=cpp

