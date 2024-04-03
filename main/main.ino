#include <mutex>
#include <thread>

#include <ESP32Servo.h>

#include "Arduino.h"

// front diff object
Servo frontDiff;
#define frontDiffPin 1

// rear diff object
Servo rearDiff;
#define rearDiffPin 2

// shifter object
Servo shifter;
#define shifterPin 3

#define servo_triggered 90
#define servo_centured 0
#define hex_size 4

#define PWM_SRC_1 1
#define PWM_SRC_2 2

// PWM
volatile std::array<unsigned long, 2> isrPulsewidth = {0, 0};     // Define Interrupt Service Routine (ISR) pulsewidth
unsigned std::array<long, 2> pulseWidth = {0, 0};                 // Define Pulsewidth variable

// shared parameter
std::mutex global_mutex;
uint32_t global_signal = 0x00000000;

// Interrupt Service Routine (ISR)
void calcPulsewidth_src1()                    
{
  static unsigned long pulseStartTime_src_1;   // Start time variable
  
  if (digitalRead(INPUT_PIN) == HIGH)    // If the change was a RISING edge
  {
    pulseStartTime_src_1 = micros();           // Store the start time (in microseconds)
  }
  else                                   // If the change was a FALLING edge
  {        
    isrPulsewidth[0] = micros() - pulseStartTime_src_1;    // Calculate the pulsewidth
  }
}

// Interrupt Service Routine (ISR)
void calcPulsewidth_src2()                    
{
  static unsigned long pulseStartTime_src_2;   // Start time variable
  
  if (digitalRead(INPUT_PIN) == HIGH)    // If the change was a RISING edge
  {
    pulseStartTime_src_2 = micros();           // Store the start time (in microseconds)
  }
  else                                   // If the change was a FALLING edge
  {        
    isrPulsewidth[1] = micros() - pulseStartTime_src_2;    // Calculate the pulsewidth
  }
}

/*
Control servos depending of single uint32_t value
front diff  -> 0x00 00 00 0F
rear diff   -> 0x00 00 00 F0
shifter     -> 0x00 00 0F 00
*/
void handle_signal(uint32_t *signal){
  /*############################################################################*/
  /*#                           front diff control                             #*/
  /*############################################################################*/
  if (*signal & 0x0000000F != 0x00000000){
    uint8_t value = uint8_t(*signal & 0x0000000F);
    switch (value) {
      case 1:
        frontDiff.write(servo_triggered);
        break;
      case 2:
        frontDiff.write(servo_centured);
      default:
        continue;
    }
    *signal = *signal & !0x0000000F;
  }
  /*############################################################################*/
  /*#                           rear diff control                              #*/
  /*############################################################################*/
  if (*signal & 0x000000F0 == 0x00000000){
    uint8_t value = uint8_t((*signal & 0x000000F0) >> (1 * hex_size));
    switch (value) {
      case 1:
        rearDiff.write(servo_triggered);
        break;
      case 2:
        rearDiff.write(servo_centured);
        break;
      default:
        continue;
    }
    *signal = *signal & !0x000000F0;
  }
  /*############################################################################*/
  /*#                             shifter control                              #*/
  /*############################################################################*/
  if (*signal & 0x00000F00 == 0x00000000){
    uint8_t value = uint8_t((*signal & 0x00000F00) >> (2 * hex_size));
    switch (value) {
      case 1:
        shifter.write(servo_triggered);
        break;
      case 2:
        shifter.write(servo_centured);
        break;
      default:
        continue;
    }
    /*############################################################################*/
  }
  *signal = *signal & !0x00000F00;
}

// Servo handler thread will control the position of all of the servos
void servo_handler(void){
  uint32_t local_signal = 0x00000000;
  while(1){
    global_mutex.lock();
    if (global_signal != 0x00000000){
      local_signal = global_signal;
      global_signal = 0x00000000;
    }
    global_mutex.unlock();
    if (local_signal != 0x00000000){
      handle_signal(&local_signal);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

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

  attachInterrupt(digitalPinToInterrupt(PWM_SRC_1), calcPulsewidth_src1, CHANGE);   // Run the calcPulsewidth function on signal CHANGE
  attachInterrupt(digitalPinToInterrupt(PWM_SRC_2), calcPulsewidth_src2, CHANGE);   // Run the calcPulsewidth function on signal CHANGE
  
}
void loop() {
  noInterrupts();                         // Turn off interrupts
  pulseWidth = isrPulsewidth;             // Copy the isr pulsewidth to the pulsewidth variable
  interrupts();                           // Turn on interrupts
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}