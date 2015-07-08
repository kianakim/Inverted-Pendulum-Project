/* Kiana Kim
 * 6/30/15
 * Description: PID implementation w/ using encoder mounted on pendulum
 * to read and make adjustments based on the position of the pendulum.
 * help from "Improving the Beginner's PID" blog post
 */

#include <Encoder.h>

Encoder myEnc (2,3);
float vout = 190;

//PID variables
unsigned long lastTime;
double input, output, error;
double setpoint = 0;
double kp = 9,
ki = 0.5,
kd =  5; //5
double errSum, lastErr;
double errArray[50];
int sampleTime = 50;
int outputEdit;

int controlDirection = 1; //0 = left, 1 = right

void setup() {
  pinMode(6, OUTPUT);
  analogWrite(6,190);
  Serial.begin(9600);
}

void loop() {
  //long input = myEnc.read();
  determineDirection();
  Compute();
  limitVoltage();

  analogWrite(6,vout);
  //Serial.println(vout);
  //Serial.println(error);
  //Serial.println(output);
  Serial.println(errSum);
  //Serial.println(outputEdit);
}

void Compute() {
  unsigned long now = millis();
  long input = myEnc.read();
  int timeChange = (now - lastTime);
  if(timeChange >= sampleTime) {
    lastErr = input;
    error = (setpoint - lastErr); //changed create 'error' variable in Compute to use prev stated variable
    int i;
    for (i=0; i>50; i=i+1){
      errArray[i] = error;
      if(i>50){
        i = 0;
      }
    } 
    errSum = error + errArray[i]; //that's not gonna work. I need the last 10 values. still working on integral number
    double dErr = (error - lastErr);

    output = kp*error + ki*errSum + kd*dErr;
    outputEdit = (output/1000)*128; //take out int for tetsing
    if(controlDirection == 0)
      vout = 210 + outputEdit; //changed from 128
    else
      vout = 0 + outputEdit;

    lastErr = error;
    lastTime = now;

    //Serial.println(error);
    //Serial.println(vout);
    //Serial.println(output);
    //Serial.println(outputEdit);
    
  }
}

void determineDirection() {
  if(error > 0) //pendulum falling left
    controlDirection = 1;
  if(error < 0) //pendulum falling right
    controlDirection = 0;

  //Serial.println(controlDirection);
}

void limitVoltage() {
  if(vout > 255)
    vout = 255;
  if(vout < 1)
    vout = 1; 
}
