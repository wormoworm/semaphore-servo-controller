# semaphore-servo-controller
## What is it?
An Arduino program for controlling servos that animate model railway semaphore signals. It takes input from up to four sensors, setting the signal to the "danger" position when any sensor is activated. The servo is animated so that the signal "bounces" when returning from clear to danger, which is often the case on the prototype. A full write-up of the project can be found [here](https://www.tomstrains.co.uk/2020/automating-semaphore-signals/).
## How does it work?
The code loops around a simple state machine, waiting for input from the sensors and moving the servo accordingly.
![Image](https://www.tomstrains.co.uk/wp-content/uploads/2020/03/signal-state-machine.svg)

The main loop follows this state machine *ad-infinitum*:
```C
void loop() {
  // Wait for "danger" to be requested.
  waitForDanger();
  // Set the signal to "danger".
  setToDanger();
  // Wait for the minimum danger time.
  waitForMinimumDangerTime();
  // Also wait until "clear" is requested.
  waitForClear();
  // Return the signal to "clear".
  setToClear();
}
```
All the major aspects of the program are covered in the [write-up](https://www.tomstrains.co.uk/2020/automating-semaphore-signals/).