#include <Arduino.h>

const int encoderPinA = 2;  // Encoder channel A
const int encoderPinB = 3;  // Encoder channel B

volatile long encoderCount = 0;  // Counter for encoder position
int lastEncoderValue = 0;        // Last read value for checking direction

// PID variables
double Kp = 2.0;  // Proportional constant
double Ki = 0.1;  // Integral constant
double Kd = 0.1;  // Derivative constant

double targetAngle = 0.0;    // Target angle in degrees
double currentAngle = 0.0;   // Current angle in degrees
double error = 0.0;          // Current error
double previousError = 0.0;  // Previous error
double integral = 0.0;       // Integral term
double derivative = 0.0;     // Derivative term

void readEncoder();

void setup() {
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  
  // Attach interrupts for the encoder pin A
  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoder, CHANGE);
  
  Serial.begin(9600);
}

void loop() {
  // Update the current angle (map encoder count to -360 to 360 degrees)
  currentAngle = map(encoderCount, -60, 60, -360, 360);
  
  // Read the target angle from serial monitor
  if (Serial.available() > 0) {
    targetAngle = Serial.parseInt();  // Set the target angle
  }
  
  // Calculate PID control
  error = targetAngle - currentAngle;
  integral += error;  // Sum the errors
  derivative = error - previousError;  // Rate of change of the error
  
  // Compute PID output
  double pidOutput = Kp * error + Ki * integral + Kd * derivative;
  
  // Apply the PID output to the system (for this case, we will just print it out)
  // In a real system, this value would be used to control a motor or actuator
  
  // Update the previous error for the next loop iteration
  previousError = error;

  // Print the results
  Serial.print("Target Angle: ");
  Serial.print(targetAngle);
  Serial.print("  Current Angle: ");
  Serial.print(currentAngle);
  Serial.print("  PID Output: ");
  Serial.println(pidOutput);

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
