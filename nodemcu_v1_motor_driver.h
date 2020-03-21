/**
   @file adafruit_motor_driver.h
   @brief Motor device driver for the Adafruit motor shield.
   @author Miguel Grinberg
*/

/**
   @file adafruit_motor_driver.h
   @brief Motor device driver for the Adafruit motor shield.
   @author Miguel Grinberg
   D1~D4 is used by motor A and B
*/

#include "motor_driver.h"

namespace Roach {
class Motor : public MotorDriver {
  public:
    /*
       @brief Class constructor.
       @param number the DC motor number to control, from 1 to 4.
    */
    Motor(int pwmPin, int dirPin)
      : MotorDriver(), currentSpeed(0)
    {
      pwm = pwmPin;
      dir = dirPin;
      pinMode(pwm, OUTPUT);
      pinMode(dir, OUTPUT);
      digitalWrite(pwm, LOW);
      digitalWrite(dir, HIGH);
    }

    void setSpeed(int speed) {
      currentSpeed = speed;
      //if (speed == 0) {
      //  analogWrite(pwm, speed * 4); // speed: 0~255, PWMA: 0~1023
      //}
      if (speed >= 0) {
        digitalWrite(dir, HIGH);
        // analogWrite(pwm, speed * 4);
      }
      else {
        digitalWrite(dir, LOW);
        // log("negtive");
        // analogWrite(pwm, -speed * 4); // 負負得正
      }
      analogWrite(pwm, abs(speed) * 4);
    }

    int getSpeed() const
    {
      return currentSpeed;
    }

  private:
    uint8_t dir;
    uint8_t pwm;
    int currentSpeed;
};
};

