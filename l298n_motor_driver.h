#include "motor_driver.h"

namespace Roach {
class Motor : public MotorDriver {
  public:
    /*
       @brief Class constructor.
       @param number the DC motor number to control, from 1 to 4.
    */
    Motor(int PinA, int PinB)
      : MotorDriver(), currentSpeed(0)
    {
      In1Or3 = PinA;
      In2Or4 = PinB;
      pinMode(In1Or3, OUTPUT);  // left or right motor 
      pinMode(In2Or4, OUTPUT);     // In2 or In4
      // digitalWrite(dir, HIGH);
    }

    void setSpeed(int speed) {
      if (speed >= 0) {
        digitalWrite(In1Or3, HIGH);
        digitalWrite(In2Or4, LOW);
      } else {
        digitalWrite(In1Or3, LOW);
        digitalWrite(In1Or3, HIGH);
      }
    }

    int getSpeed() const
    {
      return currentSpeed;
    }

  private:
    uint8_t In1Or3;
    uint8_t In2Or4;
    int currentSpeed;
};
};

