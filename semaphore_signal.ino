#include <Servo.h>
#include <math.h>

int pos = 0;    // variable to store the servo position

/********** BEGIN CONFIG **********/

// Pins
#define PIN_TRIGGER 4
#define PIN_SERVO 9

// Servo properties
#define SERVO_RANGE_DEGREES 180
#define SERVO_MICROSECONDS_MIN 700
#define SERVO_MICROSECONDS_MAX 2300
#define SWEEP_STEP_INTERVAL 10

// Servo positions
#define SERVO_ANGLE_CLEAR 125
#define SERVO_ANGLE_DANGER 80

/***** Timing *****/
// The minimum
#define MINIMUM_SET_DANGER_DELAY_MS 1000
#define MAXIMUM_SET_DANGER_DELAY_MS 3000
// The minimum time the signal servo will remain in the "danger" position.
#define MINIMUM_DANGER_TIME_MS 5000
// How long to wait before returning to "clear" after the trigger has been released. This prevents the signal servo from returning to "clear" if the trigger is released and then set again within a short time.
#define DANGER_GRACE_TIME_MS 2000

/********** END CONFIG **********/

/***** Global variables *****/
// Servo and ranges
Servo signalServo;
const int servoRangeMicroseconds = SERVO_MICROSECONDS_MAX - SERVO_MICROSECONDS_MIN;

/***** Interpolation functions *****/
float bounce(float t) {
  return t * t * 8.0;
}

float linearInterpolator(float fraction) {
  return fraction;
}

float accelerateDecelerateInterpolator(float fraction) {
  return (float)(cos((fraction + 1) * PI) / 2.0) + 0.5;
}

float getPowIn(float fraction, double power) {
  return (float) pow(fraction, power);
}

float getPowOut(float fraction, double power) {
  return (float) ((float) 1 - pow(1 - fraction, power));
}

float getPowInOut(float fraction, int power) {
  if ((fraction *= 2) < 1) {
      return (float) (0.5 * pow(fraction, power));
  }
  return (float) (1 - 0.5 * abs(pow(2 - fraction, power)));
}

float quadraticInInterpolator(float fraction) {
  return getPowIn(fraction, 2);
}

float cubicInInterpolator(float fraction) {
  return getPowIn(fraction, 3);
}

float quarticInInterpolator(float fraction) {
  return getPowIn(fraction, 4);
}

float quadraticOutInterpolator(float fraction) {
  return getPowOut(fraction, 2);
}

float cubicOutInterpolator(float fraction) {
  return getPowOut(fraction, 3);
}

float quarticOutInterpolator(float fraction) {
  return getPowOut(fraction, 4);
}

float quadraticInOutInterpolator(float fraction) {
  return getPowInOut(fraction, 2);
}

float cubicInOutInterpolator(float fraction) {
  return getPowInOut(fraction, 3);
}

float quarticInOutInterpolator(float fraction) {
  return getPowInOut(fraction, 4);
}

float circularInterpolator(float fraction) {
  if ((fraction *= 2) < 1) {
    return (float) (-0.5 * (sqrt(1 - fraction * fraction) - 1));
  }
  return (float) (0.5 * (sqrt(1 - (fraction -= 2) * fraction) + 1));
}

float getCurveAdjustment(float x, float energy){
    return -(2 * (1 - x) * 1 * x * energy + x * x) + 1;
}

float bounceCountInterpolator(float fraction, int nBounces, float energyFactor) {
  return (float) (1 + (-abs(cos(fraction * 10 * nBounces / PI)) * getCurveAdjustment(fraction, energyFactor + 0.5)));
}

float oneBounceInterpolator(float fraction) {
  return bounceCountInterpolator(fraction, 1, -1);
}

float twoBounceInterpolator(float fraction) {
  return bounceCountInterpolator(fraction, 2, -0.75);
}

float threeBounceInterpolator(float fraction) {
  return bounceCountInterpolator(fraction, 3, -0.5);
}

float fixedBounceInterpolatorOne(float fraction) {
  if (fraction < 1 / 2.75) {
      return (float) (7.5625 * fraction * fraction);
  } else if (fraction < 2 / 2.75) {
      return (float) (7.5625 * (fraction -= 1.5 / 2.75) * fraction + 0.75);
  } else if (fraction < 2.5 / 2.75) {
      return (float) (7.5625 * (fraction -= 2.25 / 2.75) * fraction + 0.9375);
  } else {
      return (float) (7.5625 * (fraction -= 2.625 / 2.75) * fraction + 0.984375);
  }
}

float fixedBounceInterpolatorTwo(float fraction) {
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

float compoundInterpolator(float fraction, float (*interpolatorOne)(float), float (*interpolatorTwo)(float)) {
  if(fraction < 0.5) {
    return (*interpolatorOne)(fraction * 2) / 2;
  }
  else{
    return 0.5 + ((*interpolatorTwo)((fraction - 0.5) * 2) / 2);
  }
}

float cubic_in_out_quartic_in_interpolator(float fraction) {
  return compoundInterpolator(fraction, cubicInOutInterpolator, quarticInInterpolator);
}

/***** Other functions *****/

boolean isPinHigh(int pin) { 
  return digitalRead(pin) == HIGH;
}

boolean isDangerSet() {
  return isPinHigh(PIN_TRIGGER);
}

void waitForDanger() {
  while(!isDangerSet()){}
  return;
}

void waitForClear() {
  // Wait until danger is no longer set.
  while(isDangerSet()){}
  // After clear is set, delay an for an additional time. This filters out transient clear events.
  waitForAdditionalDangerTime();
  // Check if danger is still set.
  if(isDangerSet()){  // If danger has been set during the additional time, recurse back and wait for clear to be set again.
    waitForClear();
  }
  else {  //If clear is still set after the additional time, return.
    return;
  }
}

int getRandomDelayMs(int minimumDelayMs, int maximumDelayMs) {
  int delayRange = maximumDelayMs - minimumDelayMs;
  return minimumDelayMs + (rand() % delayRange);
}

void randomDelay(int minimumDelayMs, int maximumDelayMs) {
  delay(getRandomDelayMs(minimumDelayMs, maximumDelayMs));
}

void waitForMinimumDangerTime() {
  delay(MINIMUM_DANGER_TIME_MS - DANGER_GRACE_TIME_MS);
}

void waitForRandomSetDangerTime() {
  randomDelay(MINIMUM_SET_DANGER_DELAY_MS, MAXIMUM_SET_DANGER_DELAY_MS);
}

void waitForAdditionalDangerTime() {
  delay(DANGER_GRACE_TIME_MS);
}

int degreesToMicroseconds(float degrees) {
    return SERVO_MICROSECONDS_MIN + ((degrees / SERVO_RANGE_DEGREES) * servoRangeMicroseconds);
}

void setServo(Servo servo, float degrees) {
  // Serial.println("Setting angle: " + String(degrees, 2));
  servo.writeMicroseconds(degreesToMicroseconds(degrees));
}

void animateServo(int startAngle, int endAngle, int durationMs, float (*function)(float)) {
  int sweepAngle = endAngle - startAngle;
  int nSteps = round((float) durationMs / (float) SWEEP_STEP_INTERVAL);
  float stepAngle = (float) sweepAngle / (float) nSteps;

  float sweepFraction, interpolatedSweepFraction;
  for (int i=0; i<=nSteps; i++) {
    sweepFraction = (float) i / (float) nSteps;
    interpolatedSweepFraction = (*function)(sweepFraction);
    setServo(signalServo, startAngle + (sweepAngle * interpolatedSweepFraction));
    delay(SWEEP_STEP_INTERVAL);
  }
}

void setToClear() {
  // Pick a random animation from a set of 4.
  int animationNumber = rand() % 4;
  switch(animationNumber) {
    default:
    case 0:   // Animation 1: Cubic in-out. Smooth acceleration and deceleration.
      animateServo(SERVO_ANGLE_DANGER, SERVO_ANGLE_CLEAR, getRandomDelayMs(1500, 2000), cubicInOutInterpolator);
      break;
    case 1:   // Animation 2: Quartic in-out. Smooth acceleration and deceleration, more pronounced change than animation 1.
      animateServo(SERVO_ANGLE_DANGER, SERVO_ANGLE_CLEAR, getRandomDelayMs(1500, 2200), quarticInOutInterpolator);
      break;
    case 2:   // Animation 3: Quartic in. Smooth acceleration as with animation 2, but with an abrupt stop.
      animateServo(SERVO_ANGLE_DANGER, SERVO_ANGLE_CLEAR, getRandomDelayMs(1700, 2200), quarticInInterpolator);
      break;
    case 3:   // Animation 4: Compound (quartic-in-out followed by quartic in). Equivalent to a mix of animation 1 followed by animation 3.
      animateServo(SERVO_ANGLE_DANGER, SERVO_ANGLE_CLEAR, getRandomDelayMs(2000, 2500), cubic_in_out_quartic_in_interpolator);
      break;
  }
}

void setToDanger() {
  // Pick a random animation from a set of 4.
  int animationNumber = rand() % 4;
  switch(animationNumber) {
    case 0:   // Animation 1: One bounce, fast decay.
      animateServo(SERVO_ANGLE_CLEAR, SERVO_ANGLE_DANGER, getRandomDelayMs(1200, 1400), oneBounceInterpolator);
      break;
    case 1:   // Animation 2: Two bounces, medium decay.
      animateServo(SERVO_ANGLE_CLEAR, SERVO_ANGLE_DANGER, getRandomDelayMs(1300, 1500), twoBounceInterpolator);
      break;
    case 2:   // Animation 3: Three bounces, slow decay.
      animateServo(SERVO_ANGLE_CLEAR, SERVO_ANGLE_DANGER, getRandomDelayMs(1400, 1600), threeBounceInterpolator);
      break;
    default:
    case 3:   // Animation 4: Three bounces, alternative algorithm.
      animateServo(SERVO_ANGLE_CLEAR, SERVO_ANGLE_DANGER, getRandomDelayMs(1300, 1600), fixedBounceInterpolatorOne);
      break;
  }
}

void setup() {
  // Debug only
  // Serial.begin(115200);
  // Serial.print("Hello World");

  // Setup input pin(s)
  pinMode(4, INPUT_PULLUP);

  // Setup the servo(s) for the signal(s)
  signalServo.attach(PIN_SERVO);
  setServo(signalServo, SERVO_ANGLE_CLEAR);    // Initialise the signal to the "clear" position
}

void loop() {
  // Wait for "danger" to be requested
  waitForDanger();
  // Wait a for a random interval before setting the signal.
  waitForRandomSetDangerTime();
  // Set the signal to "danger"
  setToDanger();
  // Wait for the minimum danger time
  waitForMinimumDangerTime();
  // Also wait until "clear" is requested
  waitForClear();
  // Return the signal to "clear"
  setToClear();
}