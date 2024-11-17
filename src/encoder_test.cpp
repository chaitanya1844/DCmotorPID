// Pins connected to the encoder
#include <Arduino.h>
const int encoderPinA = 2;  // Encoder channel A
const int encoderPinB = 3;  // Encoder channel B

volatile long encoderCount = 0; // Counter for encoder position
int lastEncoderValue = 0; // Last read value for checking direction
void readEncoder() ;
void setup() {
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  
  // Attach interrupts for the encoder pin A
  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), readEncoder, RISING);
  Serial.begin(9600);
}

void loop() {
  // Map the encoder count to -360 to 360 degrees
  //Serial.println(encoderCount);
  int degrees = map(encoderCount, -60, 60, -360, 360);
  Serial.println(degrees);

  // Short delay for stable readings
  delay(100);
}

void readEncoder() {
  // Read the current state of encoder pins
  int A = digitalRead(encoderPinA);
  int B = digitalRead(encoderPinB);
  
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
