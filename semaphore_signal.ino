#include <Servo.h>
#include <math.h>

Servo myservo;

int pos = 0;    // variable to store the servo position
int sweep_step_interval = 10;   //Update the servo every 10ms (100Hz).

// General servo parameters
const int servo_min_microseconds = 700;
const int servo_max_microseconds = 2300;
const int servo_range_microseconds = servo_max_microseconds - servo_min_microseconds;
const int servo_range_degrees = 180;

// For better bounce interpolator
const int n_bounces = 3;
const float energy_factor = 0;
const float energy = energy_factor + 0.5;

float bounce(float t) {
  return t * t * 8.0;
}


float bounce_interpolate(float fraction) {
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

float better_bounce_interpolate(float fraction) {
  return (float) (1 + (-abs(cos(fraction * 10 * n_bounces /PI)) * getCurveAdjustment(fraction)));
}

int degrees_to_microseconds(float degrees) {
  return servo_min_microseconds + ((degrees / servo_range_degrees) * servo_range_microseconds);
}

void set_servo_degrees(Servo servo, float degrees) {
  // Serial.println("Setting angle: " + String(degrees, 2));
  servo.writeMicroseconds(degrees_to_microseconds(degrees));
}

void sweep_servo(int start_angle, int end_angle, int duration_ms) {
  int sweep_angle = end_angle - start_angle;
  int n_steps = round((float) duration_ms / (float) sweep_step_interval);
  float step_angle = (float) sweep_angle / (float) n_steps;

  float sweep_fraction;
  for (int i=0; i<=n_steps; i++) {
    sweep_fraction = (float) i / (float) n_steps;
    float interpolated_sweep_fraction = better_bounce_interpolate(sweep_fraction);
    // Serial.println("Interpolated fraction: " + String(interpolated_sweep_fraction, 5));
    //myservo.write(start_angle + (round(sweep_angle * interpolated_sweep_fraction)));
    set_servo_degrees(myservo, start_angle + (sweep_angle * interpolated_sweep_fraction));
    delay(sweep_step_interval);
  }
}

void signal_up() {
  sweep_servo(80, 125, 1500);
}

void signal_down() {
  sweep_servo(125, 80, 1500);
}

void setup() {
  Serial.begin(115200);
  Serial.print("Hello World");
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  signal_up();
  delay(2000);
  signal_down();
  delay(2000);
}
