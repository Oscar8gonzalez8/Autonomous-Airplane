/*****************************************************************
   This program is designed to read a servo command, and, based on
   its value, write a servo command
*/

#include <Servo.h>

#define AUTO_SWITCH_IN (6) //GP6 is Pin 9
#define AUTO_SWITCH_OUT (13) //GP13 is Pin 17
#define LEFT_ELEVON_PIN (11) //GP11 is pin 15
#define A_SW_2 (15) //GP15 is pin 20


Servo leftElevon;

void setup() {
  Serial.begin(9600);
  pinMode(AUTO_SWITCH_IN, INPUT);
  pinMode(AUTO_SWITCH_OUT, OUTPUT);
  pinMode(LEFT_ELEVON_PIN, OUTPUT);
  pinMode(A_SW_2, OUTPUT);
  
  //attach the servo
  leftElevon.attach(LEFT_ELEVON_PIN);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(2000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
}

void loop() {
  unsigned long timeout = 50000; //(us) time to wait to receive a PWM signal before moving on
  unsigned long pwm_duration = pulseIn(AUTO_SWITCH_IN, HIGH);
  Serial.println(pwm_duration);

  //If the Auto switch is on, output the PWM command
  if (pwm_duration > 1700){
    digitalWrite(AUTO_SWITCH_OUT, LOW);
    digitalWrite(A_SW_2, HIGH);
    int pos = 0;
    for (pos = 90; pos <= 160; pos++){
      leftElevon.write(pos);
      Serial.println(pos);
      delay(15);
    }
    for (pos = 160; pos >= 90; pos--){
      leftElevon.write(pos);
      Serial.println(pos);
      delay(15);
    }
  }
  else{
    digitalWrite(A_SW_2, LOW);
    digitalWrite(AUTO_SWITCH_OUT, HIGH);
  }
  

}
