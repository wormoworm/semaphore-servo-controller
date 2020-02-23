#include <Servo.h>
#include <math.h>


int pos = 0;    // variable to store the servo position
int sweepStepInterval = 10;   //Update the servo every 10ms (100Hz).

// Track sensor parameters
#define PIN_SET_DANGER 4

// Servo and parameters
Servo signalServo;
#define PIN_SERVO 9
const int servoMinMicroseconds = 700;
const int servoMaxMicroseconds = 2300;
const int servoRangeMicroseconds = servoMaxMicroseconds - servoMinMicroseconds;
const int servoRangeDegrees = 180;

// Timing parameters
#define MINIMUM_DANGER_TIME_MS 5000

// For better bounce interpolator
const int nBounces = 3;
const float energyFactor = 0;
const float energy = energyFactor + 0.5;

float bounce(float t) {
  return t * t * 8.0;
}


float bounceInterpolate(float fraction) {
  // Taken from Android BounceInterpolator
  // _b(t) = t * t * 8
  // bs(t) = _b(t) for t < 0.3535
  // bs(t) = _b(t - 0.54719) + 0.7 for t < 0.7408
  // bs(t) = _b(t - 0.8526) + 0.9 for t < 0.9644
  // bs(t) = _b(t - 1.0435) + 0.95 for t <= 1.0
  // b(t) = bs(t * 1.1226)
  fraction *= 1.1226;
  if (fraction < 0.3535) return bounce(fraction);
  else if (fraction < 0.7408) return bounce(fraction - 0.54719) + 0.7;
  else if (fraction < 0.9644) return bounce(fraction - 0.8526) + 0.9;
  else return bounce(fraction - 1.0435) + 0.95;
}

float getCurveAdjustment(float  x){
    return -(2 * (1 - x) * 1 * x * energy + x * x) + 1;
  }

float betterBounceInterpolate(float fraction) {
  return (float) (1 + (-abs(cos(fraction * 10 * nBounces /PI)) * getCurveAdjustment(fraction)));
}

int degreesToMicroseconds(float degrees) {
  return servoMinMicroseconds + ((degrees / servoRangeDegrees) * servoRangeMicroseconds);
}

boolean isPinHigh(int pin) { 
  return digitalRead(pin) == HIGH;
}

boolean isDangerSet() {
  return isPinHigh(PIN_SET_DANGER);
}

void waitForDanger() {
  while(!isDangerSet()){}
  return;
}

void waitForClear() {
  while(isDangerSet()){}
  return;
}

void waitForMinimumDangerTime(){
  delay(MINIMUM_DANGER_TIME_MS);
}

void setServoDegrees(Servo servo, float degrees) {
  // Serial.println("Setting angle: " + String(degrees, 2));
  servo.writeMicroseconds(degreesToMicroseconds(degrees));
}

void sweepServo(int startAngle, int endAngle, int durationMs) {
  int sweepAngle = endAngle - startAngle;
  int nSteps = round((float) durationMs / (float) sweepStepInterval);
  float stepAngle = (float) sweepAngle / (float) nSteps;

  float sweepFraction;
  for (int i=0; i<=nSteps; i++) {
    sweepFraction = (float) i / (float) nSteps;
    float interpolatedSweepFraction = betterBounceInterpolate(sweepFraction);
    // Serial.println("Interpolated fraction: " + String(interpolated_sweep_fraction, 5));
    //myservo.write(start_angle + (round(sweep_angle * interpolated_sweep_fraction)));
    setServoDegrees(signalServo, startAngle + (sweepAngle * interpolatedSweepFraction));
    delay(sweepStepInterval);
  }
}

void setToClear() {
  sweepServo(80, 125, 1300);
}

void setToDanger() {
  sweepServo(125, 80, 1300);
}

void setup() {
  // Setup input pin(s)
  pinMode(4, INPUT);

  // Setup the servo(s) for the signal(s)
  signalServo.attach(PIN_SERVO);
  setToClear();    // Initialise the signal to the "clear" position

  // Debug only
  Serial.begin(115200);
  Serial.print("Hello World");
}

void loop() {
  // Wait for "danger" to be requested
  waitForDanger();
  // Set the signal to "danger"
  setToDanger();
  // Wait for the minimum danger time
  delay(MINIMUM_DANGER_TIME_MS);
  // Also wait until "clear" is requested
  waitForClear();
  // Return the signal to "clear"
  setToClear();
}