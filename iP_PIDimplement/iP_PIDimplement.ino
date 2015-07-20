/* Kiana Kim
 * 6/30/15
 * Description: PID implementation w/ using encoder mounted on pendulum
 * to read and make adjustments based on the position of the pendulum.
 * help from "Improving the Beginner's PID" blog post
 */
#include <Encoder.h>

Encoder myEnc (2,3); //sets Encoder ports

//set variables
float vout;
unsigned long lastTime;
int sampleTime = 50; //how often to run Compute(), check encoder
long input, output, outputEdit, error; //output is direct value from PID equation, error is setpoint-input
int setpoint = 0; //default setpoint is 0, upright - can be changed in changeSetpoint()
double kp = 140, // 'P' value
ki = 2, // 'I' value
kd = 30; // 'D' value
long errSum, errArray[100], lastErr; //variables used for computing Integral and Derivative parts
int counter = 0; //part of integral calculation
int rotationNum; //part of changeSetpoint(), used to reset setpoint if >360 degrees
int controlDirection = 1; //0 = left, 1 = right
unsigned long now;

void setup() {
  pinMode(6, OUTPUT);
  analogWrite(6,190);
  Serial.begin(9600);
}

void loop() {
  determineDirection(); //check direction falling based on encoder value
  changeSetpoint(); //change setpoint if pendulum has rotated >360 degrees
  Compute(); //Compute output based on PID algorithm evert 50 ms
  limitVoltage(); //limit voltage to 0-255

  analogWrite(6,vout); //send calculated voltage to power module
  //Serial.println(vout);
  Serial.println(error);
  Serial.println(output);
  //Serial.println(errArray[1]);
  //Serial.println(errSum);
  //Serial.println(setpoint);
}

void Compute() {
  if(counter >= 100)
    counter = 0; 
  now = millis();
  input = myEnc.read(); //read current encoder value
  int timeChange = (now - lastTime);
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
      double dErr = (error - lastErr);

      output = kp*error + ki*errSum + kd*dErr;
      //outputEdit = (output/100)*128; //scale the output
      if(controlDirection == 0)
        vout = 180 + output; //changed from 128 because of offset 240
      if(controlDirection == 1)
        vout = 0 + output;
  
      lastErr = error;
      lastTime = now;
      counter++;
  
      //Serial.println(error);
      //Serial.println(vout);
      //Serial.println(output);
      //Serial.println(outputEdit)
  }
}

void changeSetpoint() { //setpoint will change back once it's not 360
  if(abs(error) > 3000) //3000 encoder counts = about 270 degrees
    rotationNum = input/3000; 
    setpoint = rotationNum*4095; //4095 encoder counts = 360 degrees
}

void determineDirection() {
  if(error > 0) //pendulum falling left
    controlDirection = 1;
  if(error < 0) //pendulum falling right
    controlDirection = 0;
}

void limitVoltage() {
  if(vout > 255)
    vout = 255;
  if(vout < 1)
    vout = 1; 
  if(abs(input) > 1000 && abs(input) < 3000)
    vout = 128;    
}
