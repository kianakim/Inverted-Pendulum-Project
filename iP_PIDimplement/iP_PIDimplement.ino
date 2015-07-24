/* Kiana Kim
 * 6/30/15
 * Description: Demo software for Quanser Inverted Pendulum kit. Implements PID
 * control to track pendulum and cart position with encoders and create response
 * based on PID algorithm. 
 */

#include <Encoder.h>

Encoder myEnc (3,4); //pendulum encoder
//Encoder myEnc2 (2,5); //cart encoder

//set variables
float vout;
unsigned long lastTime;
int sampleTime = 3; //how often to run Compute(), check encoder
long input, output, outputEdit, error; //output is direct value from PID equation, error is setpoint-input
int setpoint = 0; //default setpoint is 0, upright - can be changed in changeSetpoint()
double kp = 200, // 'P' value - 200
ki = 3, // 'I' value - 3
kd = 150; // 'D' value - 150
long errSum, errArray[100], lastErr, dErr, dErrArray[5]; //variables used for computing Integral and Derivative parts
int counter = 0, counter2 = 0; //part of integral calculation
int rotationNum; //part of changeSetpoint(), used to reset setpoint if >360 degrees
int controlDirection = 1; //0 = left, 1 = right
unsigned long now;

void setup() {
  pinMode(6, OUTPUT);
  analogWrite(6,190);
  //Serial.begin(9600);
}

void loop() {
  determineDirection(); //check direction falling based on encoder value
  changeSetpoint(); //change setpoint if pendulum has rotated >360 degrees
  Compute(); //Compute output based on PID algorithm evert 50 ms
  //cartPos();
  limitVoltage(); //limit voltage to 0-255

  analogWrite(6,vout); //send calculated voltage to power module
}

void Compute() {
  if(counter >= 100)
    counter = 0; 
  if(counter2 > 5)
    counter2 = 0;
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
      //double dErr = (error - lastErr);
      
       //this is not working. - if test, reinstate counter++ and if(counter)
    dErrArray[counter2] = error; //saving last 6 error values
    if(counter2 == 5) 
      dErr = error - dErrArray[0];
    else
      dErr = error - dErrArray[abs(counter2 - 5)];

    output = kp*error + ki*errSum + kd*dErr;
    if(controlDirection == 0)
      vout = 210 + output; //changed from 128 because of offset 240
    if(controlDirection == 1)
      vout = -10 + output;
  
    lastErr = error;
    lastTime = now;
    counter++;
    counter2++;
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

/*void cartPos() {
  int cartInput = myEnc2.read();
  if(cartInput > abs(1000)) {
    
  }
}
*/
void limitVoltage() {
  if(vout > 255)
    vout = 255;
  if(vout < 1)
    vout = 1; 
  if(abs(input) > 1000 && abs(input) < 3000)
    vout = 128;    
}
