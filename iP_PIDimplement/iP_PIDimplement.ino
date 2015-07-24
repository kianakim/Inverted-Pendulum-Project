/* Kiana Kim
 * 6/30/15
 * Description: Demo software for Quanser Inverted Pendulum kit. Implements PID
 * control to track pendulum and cart position with encoders and create response
 * based on PID algorithm. 
 */

#include <Encoder.h>

Encoder myEnc (3, 4); //pendulum encoder
Encoder myEnc2 (2, 5); //cart encoder

float voutP, voutC, motorPower;
int sampleTime = 3, timeChange; //how often to run Compute(), check encoder
unsigned long lastTime, lastTimeC;

//pendulum variables
long input, output, error; //output is direct value from PID equation, error is setpoint-input
int setpoint = 0; //default setpoint is 0, upright - can be changed in changeSetpoint()
double kp = 200, // 'P' value - 200
ki = 3, // 'I' value - 3
kd = 150; // 'D' value - 150
long errSum, errArray[100], dErr, dErrArray[5]; //variables used for computing Integral and Derivative parts
int counter = 0, counter2 = 0; //part of integral calculation
int controlDirection = 1; //0 = left, 1 = right
unsigned long now;

//cart variables
long inputC, outputC, errorC;
int setpointC = 0;
int counterC = 0, counterC2 = 0;
double kpC = 1,
kiC = 0,
kdC = 0;

long errSumC, errArrayC[100], dErrC, dErrArrayC[5];

void setup() {
  pinMode(6, OUTPUT);
  analogWrite(6,160);
}

void loop() {
  determineDirection(); //check direction falling based on encoder value
  resetEncoder(); //reset the encoder to 0 when input >360 degrees
  Compute(); //Compute output based on PID algorithm evert 50 ms
  cartPos(); //Keeps cart in center of gear rack using 2nd encoder
  setPower(); //add outputs from pendulum and cart PID + limit voltage to 0-255

  analogWrite(6,motorPower); //send calculated voltage to power module
}

void Compute() {
  if(counter >= 100)
    counter = 0; 
  if(counter2 > 5)
    counter2 = 0;
  now = millis();
  input = myEnc.read(); //read current encoder value
  timeChange = (now - lastTime);
  if(timeChange >= sampleTime) { //controls when to run Compute()
      // 'P' calculation
    error = (setpoint - input);
      
      // 'I' calculation
    errSum = 0; //reset errSum, so it only takes the current 100 values
    errArray[counter] = error; //set current error to the n'th element in array
    int i;
    for(i=0; i<100; i++) {
      errSum += errArray[i]; //add past 100 error values
    } 
      
      // 'D' calculation
      //double dErr = (error - lastErr);
      
       //this is not working. - if test, reinstate counter++ and if(counter)
    dErrArray[counter2] = error; //saving last 6 error values
    if(counter2 == 5) 
      dErr = error - dErrArray[0];
    else
      dErr = error - dErrArray[abs(counter2 - 5)];

    output = kp*error + ki*errSum + kd*dErr;
    if(controlDirection == 0)
      voutP = 210 + output; //changed from 128 because of offset 240
    if(controlDirection == 1)
      voutP = -10 + output;
    
    lastTime = now;
    counter++;
    counter2++;
  }
}

void resetEncoder() { //setpoint will change back once it's not 360
  if(abs(input) >= 4095) //3000 encoder counts = about 270 degrees
    myEnc.write(0);
}

void determineDirection() {
  if(error > 0) //pendulum falling left
    controlDirection = 1;
  if(error < 0) //pendulum falling right
    controlDirection = 0;
}

void cartPos() {
  if(counterC >= 100) //reset counter for cart integral term
    counter = 0;
  inputC = myEnc2.read();
  timeChange = (now - lastTimeC); //timeChange........
  if(timeChange >= sampleTime) {
      // 'P' calculation
    errorC = (setpointC - inputC);
      
      // 'I' calculation
    errSumC = 0; //reset errSum, so it only takes the current 100 values
    errArrayC[counterC] = errorC; //set current error to the n'th element in array
    int i;
    for(i=0; i<100; i++) {
      errSumC += errArrayC[i]; //add past 100 error values
    }
    
      // 'D' calculation
    dErrArrayC[counterC2] = error; //saving last 6 error values
    if(counter == 5)
      dErrC = error - dErrArrayC[0];
    else
      dErrC = error - dErrArrayC[abs(counterC2 - 5)];

    outputC = kpC*errorC + kiC*errSumC + kdC*dErrC;
    if(controlDirection == 0)
      voutC = 210 + outputC; //changed from 128 because of offset 240
    if(controlDirection == 1)
      voutC = -10 + outputC;
    
    now = lastTimeC;
    counterC++;
    counterC2++;
  }
}

void setPower() {
  motorPower = voutC + voutP;
  if(motorPower > 255)
    motorPower = 255;
  if(motorPower < 1)
    motorPower = 1; 
  //if(abs(input) > 1000 && abs(input) < 3000)
    //vout = 128;    
}
