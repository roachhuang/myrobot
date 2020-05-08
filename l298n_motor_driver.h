#include <L298N.h>
#include "motor_driver.h"

namespace Roach
{
    class Motor : public MotorDriver
    {
    public:
        Motor(uint8_t EN, uint8_t IN1, uint8_t IN2)
            :MotorDriver(), motor(EN, IN1, IN2), currentSpeed(0)
        {
        }

        void setSpeed(int speed)
        {
            currentSpeed = speed;
            if (speed == 0){
              motor.stop();
            }
            else if (speed > 0) {
                motor.setSpeed(speed);
                motor.forward();
            }
            else {
                motor.setSpeed(-speed);
                motor.backward();
            }
        }

        int getSpeed() const
        {
            return currentSpeed;
        }

    private:
        // uint8_t EN;
        // unit8_t IN1;
        // uint8_t IN2;
        L298N motor;
        int currentSpeed;
    };
};
