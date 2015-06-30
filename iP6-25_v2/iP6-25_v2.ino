/* Kiana Kim
 * corresponds with 2015-6-25 11.46.49.3gp in Dropbox
 * status: balances pendulum for short period of time. limited, not consistent
 * Uses pendulum encoder readings to set voltage based on position
 */

#include <Encoder.h>

Encoder myEnc(2, 3);
float vout = 190; //changed from 128, neutral w/ offset

void setup() {
  pinMode(6, OUTPUT);
  analogWrite(6,190);
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
}

long oldPosition  = -999;

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    //Serial.println(newPosition); //sends position info to serial
  }
  if(newPosition > 2030) vout = 1; //control left movement
  if(newPosition < 2020) vout =255; //cotnrol right
  //vout = 1800 - newPosition; //if newPosition is larger number (cw), 
  if(newPosition > 2020 && newPosition < 2070) vout = 190;
  if(newPosition < 1000 || newPosition >2700) vout = 190;
  //if(vout>255) vout = 255;
  //if(vout<1) vout = 1;
  
  analogWrite(6,vout);
  Serial.println(vout);
}
