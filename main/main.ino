
#include <ESP32Servo.h>

// front diff object
Servo frontDiff;
#define frontDiffPin 1

// rear diff object
Servo rearDiff;
#define rearDiffPin 2

// shifter object
Servo shifter;
#define shifterPin 3

void setup() {
  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2); 
	Serial.begin(115200);
	frontDiff.setPeriodHertz(330);      // Standard 50hz servo
	rearDiff.setPeriodHertz(330);      // Standard 50hz servo
	shifter.setPeriodHertz(330);      // Standard 50hz servo

  frontDiff.attach(frontDiffPin, 1000, 2000);
	rearDiff.attach(rearDiffPin, 1000, 2000);
	shifter.attach(shifterPin, 1000, 2000);

  // zero all servos
  frontDiff.write(0);
	rearDiff.write(0);
	shifter.write(0);

void loop() {

}