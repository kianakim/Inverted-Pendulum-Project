/* Kiana Kim - 6/29/15
 * PID implementation w/ Inverted Pendulum
 * attempt to use PID control to read pendulum position and adjust
 * change. help from "improving the beginner's pid"
 */

#include <Encoder.h>

Encoder myEnc (2,3);
float vout = 190; //about neutral w/ offset

//PID Variables
unsigned long lastTime;
double input, output;
double setpoint = 2070; //about upright in encoder counts
double kp, ki, kd; //do i need to set these
double errSum, lastError;
int sampleTime = 1000; //time you want in between each run of PID, 1 sec

void setup() {
  pinMode(6, OUTPUT);
  analogWrite(6,190);
  Serial.begin(9600);
}

void loop() { //main loop
  long input = myEnc.read(); //reads encoder value as 'input'
  Compute();
  SetTunings();
  
  analogWrite(6, output);
  Serial.println(output);
  //what do I do with the output? where do I put this? how is the setpoint...set? do I need to call void Compute()
  //would it be like:
  //vout = output; (with limiters 0-255)
}

void Compute() {
  unsigned long now = millis();
  
  int timeChange = (now - lastTime);
  if(timeChange >= sampleTime) { //sets the PID to run at scheduled interval
    double error = setpoint - lastError); //2070 - 
    errSum += error; //errSum = errSum + error
    double dErr = (error - lastError); 
  
    output = kp * error + ki * errSum + kd *dErr; 
    
    lastErr = error; //updates error and time
    lastTime = now;
  }
}

void SetTunings(double Kp, double Ki, double Kd) {
  double SampleTimeInSec = ((double)SampleTime)/1000;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;
}


