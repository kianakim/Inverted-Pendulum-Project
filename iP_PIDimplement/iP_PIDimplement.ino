/* Kiana Kim - 6/29/15
 * PID implementation w/ Inverted Pendulum
 * attempt to use PID control to read pendulum position and adjust
 * rate of change with Kp values
 */

#include <Encoder.h>

Encoder myEnc (2,3);
float vout = 190; //about neutral w/ offset

//PID Variables
unsigned long lastTime
double input, output, setpoint;
double kp, ki, kd;

void setup() {
  pinMode(6, OUTPUT);
  analogWrite(6,190);
  Serial.begin(9600);
}

//Take 
void loop() {
  long input = myEnc.read(); //reads encoder value as 'input'
  setpoint = 
  
}
  

