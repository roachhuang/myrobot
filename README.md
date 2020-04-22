# mosquitto-myrobot
mosquitto version of myrobot

NOTE:
don't know why it is a must to set pinmode again for each motor movemnet; otherwise, the left motor can only move on one direction.
The robot won't move as expected if running out of batteries

mosquitto.conf anonymous = false (auth) doens't work, esp8266 keeps disconnecting, so change the setting to true. 