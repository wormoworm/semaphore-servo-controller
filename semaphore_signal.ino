#include <Servo.h>
#include <math.h>

int pos = 0;    // variable to store the servo position

/********** BEGIN CONFIG **********/

// Pins
#define PIN_5V_REF A4
#define PIN_TRIGGER_A 2
#define PIN_TRIGGER_B 3
#define PIN_TRIGGER_C 4
#define PIN_TRIGGER_D 5
#define PIN_SERVO 6
#define PIN_LED_DANGER 13

#define TRIGGER_SAMPLING_INTERVAL 10
#define DANGER_SAMPLE_COUNT 50
#define CLEAR_SAMPLE_COUNT 100

// Servo properties
#define SERVO_RANGE_DEGREES 180
#define SERVO_MICROSECONDS_MIN 700
#define SERVO_MICROSECONDS_MAX 2300
#define SWEEP_STEP_INTERVAL 10

// Servo positions
#define SERVO_ANGLE_CLEAR 93
#define SERVO_ANGLE_DANGER 120

/***** Timing *****/
// Random delay
#define MINIMUM_RANDOM_DELAY_MS 100
#define MAXIMUM_RANDOM_DELAY_MS 500
// The minimum time the signal servo will remain in the "danger" position.
#define MINIMUM_DANGER_TIME_MS 2000
// Sequence delay
#define SEQUENCE_NUMBER 2
#define SEQUENCE_DELAY 3000   // 3 seconds between each signal moving.

/********** END CONFIG **********/

/***** Global variables *****/
// Servo and ranges
Servo signalServo;
int currentServoAngle = 255;

const int servoRangeMicroseconds = SERVO_MICROSECONDS_MAX - SERVO_MICROSECONDS_MIN;
const int servoMinAngle = min(SERVO_ANGLE_CLEAR, SERVO_ANGLE_DANGER);
const int servoMaxAngle = max(SERVO_ANGLE_CLEAR, SERVO_ANGLE_DANGER);

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
  return isPinHigh(PIN_TRIGGER_A) || isPinHigh(PIN_TRIGGER_B) || isPinHigh(PIN_TRIGGER_C) || isPinHigh(PIN_TRIGGER_D) ;
}

void waitForDanger() {
  int count = 0;
  while(count < DANGER_SAMPLE_COUNT){
    if(isDangerSet()){
      count++;
    }
    else count = 0;
    delay(TRIGGER_SAMPLING_INTERVAL);
  }
  return;
}

void waitForClear() {
  int count = 0;
  while(count < CLEAR_SAMPLE_COUNT){
    if(!isDangerSet()){
      count++;
    }
    else count = 0;
    delay(TRIGGER_SAMPLING_INTERVAL);
  }
  return;
}

int getRandomDelayMs(int minimumDelayMs, int maximumDelayMs) {
  int delayRange = maximumDelayMs - minimumDelayMs;
  return minimumDelayMs + (rand() % delayRange);
}

void randomDelay(int minimumDelayMs, int maximumDelayMs) {
  delay(getRandomDelayMs(minimumDelayMs, maximumDelayMs));
}

void sequenceDelay(){
  delay(SEQUENCE_NUMBER * SEQUENCE_DELAY);
}

void waitForMinimumDangerTime() {
  // Subtract the sampling time from the delay, so the signal really can return to clear after MINIMUM_DANGER_TIME_MS
  int initialDelayTime = MINIMUM_DANGER_TIME_MS - (TRIGGER_SAMPLING_INTERVAL * CLEAR_SAMPLE_COUNT);
  if(initialDelayTime < 0) initialDelayTime = 0;
  delay(initialDelayTime);
}

void waitForRandomSetDangerTime() {
  randomDelay(MINIMUM_RANDOM_DELAY_MS, MAXIMUM_RANDOM_DELAY_MS);
}

int degreesToMicroseconds(float degrees) {
    return SERVO_MICROSECONDS_MIN + ((degrees / SERVO_RANGE_DEGREES) * servoRangeMicroseconds);
}

void setServo(Servo servo, float degrees) {
  // Sanity-check the angle we are setting so we don't break the signal!
  if(degrees < servoMinAngle){
    degrees = servoMinAngle;
  }
  else if(degrees > servoMaxAngle){
    degrees = servoMaxAngle;
  }
  servo.writeMicroseconds(degreesToMicroseconds(degrees));
  currentServoAngle = degrees;
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
  // Catch the edge case where the servo may already be set to clear (this shouldn't ever happen, but it's here just in case).
  if(currentServoAngle==SERVO_ANGLE_CLEAR){
    return;
  }
  sequenceDelay();
  randomDelay(MINIMUM_RANDOM_DELAY_MS, MAXIMUM_RANDOM_DELAY_MS);
  digitalWrite(PIN_LED_DANGER, LOW);
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
  // Catch the edge case where the servo may already be set to danger (this will occur once after startup if isDangerSet() returns true during setup()).
  if(currentServoAngle==SERVO_ANGLE_DANGER){
    return;
  }
  sequenceDelay();
  randomDelay(MINIMUM_RANDOM_DELAY_MS, MAXIMUM_RANDOM_DELAY_MS);
  digitalWrite(PIN_LED_DANGER, HIGH);
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

void flash_danger_indicator(int flashes){
  if(flashes < 1){
    return;
  }
  for(int i=0; i<flashes; i++){
    digitalWrite(PIN_LED_DANGER, HIGH);
    delay(300);
    digitalWrite(PIN_LED_DANGER, LOW);
    delay(200);
  }
}

void setup() {
  pinMode(PIN_5V_REF, OUTPUT);
  digitalWrite(PIN_5V_REF, HIGH);

  // Setup the LED indicators
  pinMode(PIN_LED_DANGER, OUTPUT);

  // Flash the danger indicator 3 times.
  flash_danger_indicator(3);
  // Delay for a second and then flash the danger indicator to indicate what sequence number this controller has.
  delay(1000);
  flash_danger_indicator(SEQUENCE_NUMBER);

  // Setup input pin(s)
  pinMode(PIN_TRIGGER_A, INPUT);
  pinMode(PIN_TRIGGER_B, INPUT);
  pinMode(PIN_TRIGGER_C, INPUT);
  pinMode(PIN_TRIGGER_D, INPUT);

  // Setup the servo(s) for the signal(s)
  signalServo.attach(PIN_SERVO);

  //Set the servo to the position that matches the current startAngle. Do not animate.
  if(isDangerSet()){
    setServo(signalServo, SERVO_ANGLE_DANGER);
  }
  else{
    setServo(signalServo, SERVO_ANGLE_CLEAR);
  }
}

void loop() {
  // Wait for "danger" to be requested
  waitForDanger();
  // Wait a for a random interval before setting the signal.
  // waitForRandomSetDangerTime();
  // Set the signal to "danger"
  setToDanger();
  // Wait for the minimum danger time
  waitForMinimumDangerTime();
  // Also wait until "clear" is requested
  waitForClear();
  // Return the signal to "clear"
  setToClear();
}