/***************************************************
//ABRA ELECTRONICS
//6DOF ARM ROBOTIC
****************************************************/

#include "HCPCA9685.h"  // 설치한 헤더 파일
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);
// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You’ll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN 150 // this is the ‘minimum’ pulse length count (out of 4096)
#define SERVOMAX 600 // this is the ‘maximum’ pulse length count (out of 4096)
#define I2CAdd 0x40
HCPCA9685 HCPCA9685(I2CAdd);

// our servo # counter
uint8_t servonum0 = 0;
uint8_t servonum1 = 1;

int input=0;

Servo servo;
void setup() {
  Serial.begin(9600);
  servo.attach(servonum0);
  pwm.begin();
  pwm.setPWMFreq(30); // Analog servos run at ~60 Hz updates
}
// you can use this function if you’d like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
//void setServoPulse(uint8_t n, double pulse) {
//  double pulselength;
//  pulselength = 1000000; // 1,000,000 us per second
  
//  pulselength /= 30; // 60 Hz
//  pulselength /= 4096; // 12 bits of resolution
//  pulse *= 1000000; // convert to us
//  pulse /= pulselength;
//  pwm.setPWM(n, 0, pulse);
//}

void loop(){
  unsigned int Pos;
  if(Serial.available()){
    int vial = Serial.parseInt();

     if(vial == 0){
        for(Pos = 10; Pos < 200; Pos++)
        {
          HCPCA9685.Servo(0, Pos);
          delay(7);
        }
        delay(9);
        for(Pos = 200; Pos >= 10; Pos--)
        {
          HCPCA9685.Servo(0, Pos);
          delay(7);
        }
      } else if (vial == 1){
        for(Pos = 10; Pos < 200; Pos++)
        {
          HCPCA9685.Servo(1, Pos);
          delay(7);
        }
        delay(9);
        for(Pos = 200; Pos >= 10; Pos--)
        {
          HCPCA9685.Servo(1, Pos);
          delay(7);
        }
      } else if (vial == 2){
        for(Pos = 10; Pos < 200; Pos++)
        {
          HCPCA9685.Servo(2, Pos);
          delay(7);
        }
        delay(9);
        for(Pos = 200; Pos >= 10; Pos--)
        {
          HCPCA9685.Servo(2, Pos);
          delay(7);
        }
      } else if (vial == 3){
        for(Pos = 10; Pos < 200; Pos++)
        {
          HCPCA9685.Servo(3, Pos);
          delay(7);
        }
        delay(9);
        for(Pos = 200; Pos >= 10; Pos--)
        {
          HCPCA9685.Servo(3, Pos);
          delay(7);
        }
      } else if (vial == 4){
        for(Pos = 10; Pos < 200; Pos++)
        {
          HCPCA9685.Servo(4, Pos);
          delay(7);
        }
        delay(9);
        for(Pos = 200; Pos >= 10; Pos--)
        {
          HCPCA9685.Servo(4, Pos);
          delay(7);
        }
      } else if (vial == 5){
        for(Pos = 10; Pos < 200; Pos++)
        {
          HCPCA9685.Servo(5, Pos);
          delay(3);
        }
        delay(5);
        for(Pos = 200; Pos >= 10; Pos--)
        {
          HCPCA9685.Servo(5, Pos);
          delay(3);
        }
      } else if (vial == 6){
        for(Pos = 10; Pos < 200; Pos++)
        {
          HCPCA9685.Servo(6, Pos);
          delay(3);
        }
        delay(5);
        for(Pos = 200; Pos >= 10; Pos--)
        {
          HCPCA9685.Servo(6, Pos);
          delay(3);
        }
      } else if (vial == 7){
        for(Pos = 10; Pos < 200; Pos++)
        {
          HCPCA9685.Servo(7, Pos);
          delay(3);
        }
        delay(5);
        for(Pos = 200; Pos >= 10; Pos--)
        {
          HCPCA9685.Servo(7, Pos);
          delay(3);
        }
      } else if (vial == 8){
        for(Pos = 10; Pos < 200; Pos++)
        {
          HCPCA9685.Servo(8, Pos);
          delay(3);
        }
        delay(5);
        for(Pos = 200; Pos >= 10; Pos--)
        {
          HCPCA9685.Servo(8, Pos);
          delay(3);
        }
      } else if (vial == 9){
        for(Pos = 10; Pos < 200; Pos++)
        {
          HCPCA9685.Servo(9, Pos);
          delay(3);
        }
        delay(5);
        for(Pos = 200; Pos >= 10; Pos--)
        {
          HCPCA9685.Servo(9, Pos);
          delay(3);
        }
      }
    }
}
