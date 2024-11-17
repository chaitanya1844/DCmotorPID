#include <Arduino.h>
// This alternate version of the code does not require
// atomic.h. Instead, interrupts() and noInterrupts() 
// are used. Please use this code if your 
// platform does not support ATOMIC_BLOCK.

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 10
#define IN2 5
#define IN1 4
#define maxSpeed 255

double targetAngle = 0.0;    // Target angle in degrees
double currentAngle = 0.0; 

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;


volatile long encoderCount = 0; // Counter for encoder position
int lastEncoderValue = 0;

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void readEncoder();


void setup() {
  Serial.begin(9600);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
/*   attachInterrupt(digitalPinToInterrupt(ENCB), readEncoder, RISING); */
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  Serial.println("angle pos");


}

void loop() {
  Serial.println(encoderCount);
  currentAngle = map(encoderCount, -60, 60, 360, -360);
  
  if (Serial.available() > 0) {
    targetAngle = Serial.parseInt();  // Set the target angle
  }
  
  // set angle position
  int angle = targetAngle;
  //int angle = 250*sin(prevT/1e6);

  // PID constants
  float kp = 10;
  float kd = 0.025;
  float ki = 0.1;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = currentAngle;
  interrupts(); // turn interrupts back on
  
  // error
  int e = currentAngle - angle;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > maxSpeed ){
    pwr = maxSpeed;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;

  Serial.print(angle);
  Serial.print(" ");
  Serial.print(currentAngle);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}


void readEncoder() {
  // Read the current state of encoder pins
  int A = digitalRead(ENCA);
  int B = digitalRead(ENCB);
  
  // Determine the direction
  if (A == HIGH) {
    if (B == LOW) {
      encoderCount++;  // Clockwise
    } else {
      encoderCount--;  // Counterclockwise
    }
  } else {
    if (B == HIGH) {
      encoderCount++;  // Clockwise
    } else {
      encoderCount--;  // Counterclockwise
    }
  }
}
