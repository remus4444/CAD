#include <PID_v1.h> // import pid lib

// define pins
const int encoderPinA = 2, encoderPinB = 3;
const int stepPin = 4, dirPin = 5, thermistorPin = A0;

// Define machine constants
const float countsToDegrees = 0.183299; 
const int gearRatio = 4; 
#define RT0 10000 
#define B 3950 
#define R_SERIES 10000 

// Defines PID vars
double setpointTemp, currentTemp, thermalOffset;
double targetDialAngle, currentDialAngle, stepperOutput;

// Thermal PID
double Kp_t = 1.2, Ki_t = 0.2, Kd_t = 0.1; 
PID thermalPID(&currentTemp, &thermalOffset, &setpointTemp, Kp_t, Ki_t, Kd_t, DIRECT);

// Motion PID
double Kp_m = 0.1, Ki_m = 0.0, Kd_m = 0.0;
PID motionPID(&currentDialAngle, &stepperOutput, &targetDialAngle, Kp_m, Ki_m, Kd_m, DIRECT);

volatile long encoderCount = 0;

void setup() {
  Serial.begin(9600);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  thermalPID.SetMode(AUTOMATIC);
  thermalPID.SetOutputLimits(-50, 50); // sets correction degree limits
  
  motionPID.SetMode(AUTOMATIC);
  motionPID.SetOutputLimits(-150, 150);
}

void loop() {
  // define target
  setpointTemp = 75; // sets target temp

  // read temp
  float VRT = (5.00 / 1023.00) * analogRead(thermistorPin);
  float RT = VRT / ((5.00 - VRT) / R_SERIES);
  float ln = log(RT / RT0);
  float tempK = (1 / ((ln / B) + (1 / (25 + 273.15))));
  currentTemp = ((tempK - 273.15) * 1.8) + 32; // Convert to Fahrenheit

  // calc thermal offset
  thermalPID.Compute();

  // calc target dial angle
  float baseAngle = setpointTemp; 
  targetDialAngle = baseAngle + thermalOffset;

  // read encoder position
  currentDialAngle = encoderCount * countsToDegrees * -1;

  // move motor
  motionPID.Compute(); // use pid lib
  executeMove(stepperOutput);

  // print to serial monitor
  Serial.print("Target: "); Serial.print(setpointTemp);
  Serial.print("F | Actual: "); Serial.print(currentTemp);
  Serial.print("F | Offset: "); Serial.print(thermalOffset);
  Serial.print(" | Dial: "); Serial.println(currentDialAngle);
  
  delay(10);
}

void executeMove(double steps) {
  if (abs(steps) < 1) return; // deadzone
  
  digitalWrite(dirPin, steps > 0 ? HIGH : LOW);
  int totalSteps = abs(steps) * gearRatio;

  for (int i = 0; i < totalSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(900);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(900);
  }
}

void handleEncoder() {
  if (digitalRead(encoderPinB) == LOW) encoderCount++;
  else encoderCount--;
}