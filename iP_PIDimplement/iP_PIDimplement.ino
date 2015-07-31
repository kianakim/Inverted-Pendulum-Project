/* Kiana Kim
 * 6/30/15
 * Description: Demo software for Quanser Inverted Pendulum kit. Implements PID
 * control to track pendulum and cart position with encoders and create response
 * based on PID algorithm. 
 */

#include <Encoder.h>

Encoder myEnc2 (3,4); //pendulum encoder
Encoder myEnc (2,5); //cart encoder

float voutP, voutC, motorPower;
int sampleTime = 3, timeChange; //how often to run Compute(), check encoder
unsigned long lastTime, lastTimeC;

//pendulum PID variables
long input, output, error; //output is direct value from PID equation, error is setpoint-input
int setpoint = 0; //default setpoint is 0, upright - can be changed in changeSetpoint()
double kp = 200, // 'P' value - 200
ki = 3, // 'I' value - 3
kd = 150; // 'D' value - 150
long errSum, errArray[100], dErr, dErrArray[5]; //variables used for computing Integral and Derivative parts
int counter = 0, counter2 = 0; //part of integral calculation
int controlDirection = 1; //0 = left, 1 = right
unsigned long now;

//cart PID variables
long inputC, outputC, errorC;
int setpointC = 0;
int counterC = 0, counterC2 = 0;
double kpC = .4,
kiC = 0,
kdC = 0;
long errSumC, errArrayC[100], dErrC;
double spk = ; //setpoint variable

void setup() {
  pinMode(6, OUTPUT);
  analogWrite(6,160);
  //Serial.begin(9600);
}

void loop() {
  if(counter >= 100)
    counter = 0; 
  determineDirection(); //check direction falling based on encoder value
  resetEncoder(); //reset the encoder to 0 when input >360 degrees
  Compute(); //Compute output based on PID algorithm evert 50 ms
  cartPos(); //Keeps cart in center of gear rack using 2nd encoder
  setPower(); //add outputs from pendulum and cart PID + limit voltage to 0-255

  analogWrite(6,motorPower); //send calculated voltage to power module
  counter++;
}

void Compute() {
  now = millis();
  input = myEnc.read(); //read current encoder value
  timeChange = (now - lastTime);
  //if(timeChange >= sampleTime) { //controls when to run Compute()
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
    errArray[counter] = error; //saving last 6 error values
    if(counter == 5) 
      dErr = error - errArray[0];
    else
      dErr = error - errArray[abs(counter - 5)];

    output = kp*error + ki*errSum + kd*dErr;
    if(controlDirection == 0)
      voutP = 210 + output; //changed from 128 because of offset 240
    if(controlDirection == 1)
      voutP = -10 + output;
    
    lastTime = now;
  //}
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
  inputC = myEnc2.read(); //reads cart encoder values

  }
  /*now = millis();
  inputC = myEnc2.read(); //reads cart encoder values
  timeChange = (now - lastTimeC); //timeChange........
  if(timeChange >= sampleTime) {
      // 'P' calculation
    errorC = (setpointC - inputC);
      
      // 'I' calculation
    errSumC = 0; //reset errSum, so it only takes the current 100 values
    errArrayC[counter] = errorC; //set current error to the n'th element in array
    int i;
    for(i=0; i<100; i++) {
      errSumC += errArrayC[i]; //add past 100 error values
    }
    
      // 'D' calculation
    dErrC = errorC - errArrayC[abs(counter - 5)];

    outputC = kpC*errorC + kiC*errSumC + kdC*dErrC;
    voutC = 128 + outputC;
    /*
      voutC = 210 + outputC; //changed from 128 because of offset
    if(inputC > 0)
      voutC = -10 + outputC;
    
    now = lastTimeC;
  }
}
void setPower() {
  motorPower = voutP + voutC;
  if(motorPower > 255)
    motorPower = 255;
  if(motorPower < 1)
    motorPower = 1; 
  //if(abs(input) > 1000 && abs(input) < 3000)
    //vout = 128;    
/*
  //print status:
  Serial.print("Encoder Value: ");
  Serial.print(inputC);
  Serial.print("   Output from PDI: : ");
  Serial.print(outputC);
  Serial.print("   Motor Voltage:");
  Serial.print(motorPower);
  Serial.print("\n");
*/
}
