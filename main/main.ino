#include <mutex>
#include <thread>

#include <ESP32Servo.h>

#include "Arduino.h"

// front diff object
Servo frontDiff;
#define frontDiffPin 27

// rear diff object
Servo rearDiff;
#define rearDiffPin 26

// shifter object
Servo shifter;
#define shifterPin 25

#define servo_triggered 170
#define servo_centured 90
#define hex_size 4

#define PWM_SRC_1 17
#define PWM_SRC_2 16

#define High_PWM_Trigger 1700
#define Low_PWM_Trigger 1300
#define SIGNAL_STEP_SIZE 50

#define SERVO_PWM_CENTURE 475

/* Signal control points */
#define BOTH_DIFF_POINT -1
#define FRONT_DIFF_POINT 1
#define REAR_DIFF_POINT 3
#define SHIFTER_POINT 5

// PWM
std::mutex pwm_lock_src_1, pwm_lock_src_2;
std::array<unsigned long, 2> isrPulsewidth = {0, 0};     // Define Interrupt Service Routine (ISR) pulsewidth
std::array<unsigned long, 2> pulseWidth = {0, 0};                 // Define Pulsewidth variable

// shared parameter
std::mutex global_mutex;
uint32_t global_signal = 0x00000000;

// Interrupt Service Routine (ISR)
void calcPulsewidth_src1()                    
{
  static unsigned long pulseStartTime_src_1;   // Start time variable
  
  if (digitalRead(PWM_SRC_1) == HIGH)    // If the change was a RISING edge
  {
    pulseStartTime_src_1 = micros();           // Store the start time (in microseconds)
  }
  else                                   // If the change was a FALLING edge
  {
    pwm_lock_src_1.lock();
    isrPulsewidth[0] = micros() - pulseStartTime_src_1;    // Calculate the pulsewidth
    pwm_lock_src_1.unlock();
  }
}

// Interrupt Service Routine (ISR)
void calcPulsewidth_src2()                    
{
  static unsigned long pulseStartTime_src_2;   // Start time variable
  
  if (digitalRead(PWM_SRC_2) == HIGH)    // If the change was a RISING edge
  {
    pulseStartTime_src_2 = micros();           // Store the start time (in microseconds)
  }
  else                                   // If the change was a FALLING edge
  {
    pwm_lock_src_2.lock();
    isrPulsewidth[1] = micros() - pulseStartTime_src_2;    // Calculate the pulsewidth
    pwm_lock_src_2.unlock();
  }
}

/*
Control servos depending of single uint32_t value
front diff  -> 0x00 00 00 0F
rear diff   -> 0x00 00 00 F0
shifter     -> 0x00 00 0F 00
*/
void handle_signal(uint32_t signal){
  /*############################################################################*/
  /*#                           front diff control                             #*/
  /*############################################################################*/
  if ((signal & 0x0000000F) != 0x00000000){
    uint8_t value = uint8_t(signal & 0x0000000F);
    Serial.println("front diff value: " + String(value));
    switch (value) {
      case 1:
        frontDiff.write(155);
        break;
      case 2:
        frontDiff.write(105);
      default:
        break;
    }
    // signal = signal & !0x0000000F;
  }
  /*############################################################################*/
  /*#                           rear diff control                              #*/
  /*############################################################################*/
  if ((signal & 0x000000F0) != 0x00000000){
    uint8_t value = uint8_t((signal & 0x000000F0) >> (1 * hex_size));
    Serial.println("rear diff value: " + String(value));
    switch (value) {
      case 1:
        rearDiff.write(155);
        break;
      case 2:
        rearDiff.write(105);
        break;
      default:  
        break;
    }
    // signal = signal & !0x000000F0;
  }
  /*############################################################################*/
  /*#                             shifter control                              #*/
  /*############################################################################*/
  if ((signal & 0x00000F00) != 0x00000000){
    uint8_t value = uint8_t((signal & 0x00000F00) >> (2 * hex_size));
    Serial.println("Shifter value: " + String(value));
    switch (value) {
      case 1:
        shifter.write(servo_triggered);
        break;
      case 2:
        shifter.write(servo_centured);
        break;
      default:
        break;
    }
    /*############################################################################*/
  }
  signal = signal & !0x00000FFF;
}

// Servo handler thread will control the position of all of the servos
// void servo_handler(void){
//   uint32_t local_signal = 0x00000000;
//   while(1){
//     global_mutex.lock();
//     if (global_signal != 0x00000000){
//       local_signal = global_signal;
//       global_signal = 0x00000000;
//     }
//     global_mutex.unlock();
//     if (local_signal != 0x00000000){
//       handle_signal(&local_signal);
//     }
//     std::this_thread::sleep_for(std::chrono::milliseconds(10));
//   }
// }

// void send_signal_to_thread(uint8_t signal){
//   global_mutex.lock();
//   global_signal = signal;
// global_mutex.unlock();
// }

void setup() {
  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2); 
	Serial.begin(115200);
	frontDiff.setPeriodHertz(50);      // Standard 50hz servo
	rearDiff.setPeriodHertz(50);      // Standard 50hz servo
	shifter.setPeriodHertz(50);      // Standard 50hz servo

  frontDiff.attach(frontDiffPin, 1000, 2000);
	rearDiff.attach(rearDiffPin, 1000, 2000);
	shifter.attach(shifterPin, 1000, 2000);

  // zero all servos
  frontDiff.write(105);
	rearDiff.write(105);
	shifter.write(90);

  attachInterrupt(digitalPinToInterrupt(PWM_SRC_1), calcPulsewidth_src1, CHANGE);   // Run the calcPulsewidth function on signal CHANGE
  attachInterrupt(digitalPinToInterrupt(PWM_SRC_2), calcPulsewidth_src2, CHANGE);   // Run the calcPulsewidth function on signal CHANGE
  
}

/*
+Ve = 6v
Signal voltage = 3.3v

*/
void loop() {
  // interupt safe
  noInterrupts();                         // Turn off interrupts
  pwm_lock_src_1.lock();
  pwm_lock_src_2.lock();
  pulseWidth = isrPulsewidth;             // Copy the isr pulsewidth to the pulsewidth variable
  pwm_lock_src_1.unlock();
  pwm_lock_src_2.unlock();
  interrupts();                           // Turn on interrupts
  Serial.print(pulseWidth[0] - 1000);
  Serial.print(" ");
  Serial.println(pulseWidth[1] - 1000);
  if (pulseWidth[1] >= High_PWM_Trigger || pulseWidth[1] <= Low_PWM_Trigger){
    // trigger control for second core
    
    unsigned long selector = ((pulseWidth[0]  - 1000));
    Serial.println("Range 1: " + String(selector > 525) + "-" + String(selector <= 575) + "\t selector value: " + String(selector));
    if ((selector > (SERVO_PWM_CENTURE + (SIGNAL_STEP_SIZE * FRONT_DIFF_POINT))) && (selector <= (SERVO_PWM_CENTURE + (SIGNAL_STEP_SIZE * (FRONT_DIFF_POINT + 1))))){
      // Serial.println("Front Diff");
      if (pulseWidth[1] >= High_PWM_Trigger){
        handle_signal(0x00000002);
      }
      else {
        handle_signal(0x00000001);
      }
    }
    else if (selector > (SERVO_PWM_CENTURE + (SIGNAL_STEP_SIZE * REAR_DIFF_POINT)) && selector <= (SERVO_PWM_CENTURE + (SIGNAL_STEP_SIZE * (REAR_DIFF_POINT + 1)))){
      Serial.println("Rear Diff");
      if (pulseWidth[1] >= High_PWM_Trigger){
        
        handle_signal(0x00000020);
      }
      else {
        handle_signal(0x00000010);
      }
    }
    else if (selector > (SERVO_PWM_CENTURE + (SIGNAL_STEP_SIZE * SHIFTER_POINT)) && selector <= (SERVO_PWM_CENTURE + (SIGNAL_STEP_SIZE * (SHIFTER_POINT + 1)))){
      // Serial.println("Shifter");
      if (pulseWidth[1] >= High_PWM_Trigger){
        handle_signal(0x00000200);
      }
      else {
        handle_signal(0x00000100);
      }
    }
    else if (selector > (SERVO_PWM_CENTURE + (SIGNAL_STEP_SIZE * BOTH_DIFF_POINT)) && selector <= (SERVO_PWM_CENTURE + (SIGNAL_STEP_SIZE * (BOTH_DIFF_POINT + 1)))){
      // Serial.println("Both Diffs");
      if (pulseWidth[1] >= High_PWM_Trigger){
        handle_signal(0x00000022);
      }
      else {
        handle_signal(0x00000011);
      }
    }

  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
}