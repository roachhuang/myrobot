// the ultra sonic sensor needs 5V to work, we can connect its VCC to the Vin pin of the nodemcu motor shield
#include "distance_sensor.h"

namespace Roach {
class DistanceSensor: public DistanceSensorDriver {
  public:
    DistanceSensor(int triggerPin, int echoPin, int maxDistance)
      : DistanceSensorDriver(maxDistance)
    {
      trigger_pin = triggerPin;
      echo_pin = echoPin;
      pinMode(triggerPin, OUTPUT); // Sets the trigPin as an Output
      pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    }

    virtual unsigned int getDistance() {
      // Clears the trigPin
      digitalWrite(trigger_pin, LOW);
      delayMicroseconds(2);

      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigger_pin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigger_pin, LOW);

      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echo_pin, HIGH);

      // Calculating the distance
      return (duration * 0.034 / 2);
    }

  private:
    long duration;
    int trigger_pin;
    int echo_pin;
};
};


