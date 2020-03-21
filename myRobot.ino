
#define LOGGING
// has to be included on 1st line
#include "logging.h"

#include <ESP8266WiFi.h>
// Import required libraries
// Libs for connecting to any available wifi
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>

const char* mqtt_server = "ajoan.com";
const int mqtt_port = 1883;
// Clients
WiFiClient espClient;
PubSubClient client(espClient);


/* we use wifimanager to connect to any available wifi
  // #define CLSA
  // WiFi parameters
  #ifdef CLSA
  const char *ssid = "CLSA";
  const char *password = "1234Abcd";
  #else
  const char *ssid = "c1225";
  const char *password = "c0918321359";
  #endif
*/

// Functions
void callback(char *topic, byte *payload, unsigned int length);

// enable one of the motor shields below
//#define ENABLE_ADAFRUIT_MOTOR_DRIVER
#ifdef ENABLE_ADAFRUIT_MOTOR_DRIVER
#include "adafruit_motor_driver.h"
#define LEFT_MOTOR_INIT 3 // M3 and M4
#define RIGHT_MOTOR_INIT 4
#endif

#define ENABLE_NODEMCU_V1_MOTOR_DRIVER
#ifdef ENABLE_NODEMCU_V1_MOTOR_DRIVER
#include "nodemcu_v1_motor_driver.h"
#define LEFT_MOTOR_INIT D1, D3  // A+ and A- pwm:D1, direction: D3
#define RIGHT_MOTOR_INIT D2, D4 // B+ and B-
#endif

#include "nodemcu_ultrasonic_sensor.h"
const int trigPin = D6; //D4
const int echoPin = D8; //D3
#define MAX_DISTANCE 200
Roach::DistanceSensor dSensor(trigPin, echoPin, MAX_DISTANCE);

const uint8_t MIN_SPEED = 00;  // no less than 30%
const uint8_t MAX_SPEED = 255; // no greater than 50%
const uint8_t FIX_SPEED = 170; // right motor fix at 40% speed.

// Roach is a namespace
Roach::Motor leftMotor(LEFT_MOTOR_INIT);
Roach::Motor rightMotor(RIGHT_MOTOR_INIT);

// auto mode
unsigned long startTime;
#define irLeft D5  // A0
#define irRight D7 // A2

// Declare functions to be exposed to the API
void stop(void);
void forward(void);
void left(void);
void right(void);
void backward(void);

bool selfDriving = false; // default manual mode
uint8_t motorSpeed = 150;
uint8_t maxDist2Wall = 8; // 3cm.
uint8_t fsmDelay = 300;        // default 20ms

enum state_t
{
  // S,  // bug - to be fixed coz fms[4] has 5 state but i just put 4 states
  F,
  B,
  L,
  R
};
state_t s;
// Linked data structure
struct State
{
  void (*cmdPtr)();
  unsigned long delay; // time to delay in ms
  //unsigned long next[8];
  state_t next[8];
}; // Next if 3-bit input is 0-7
typedef const struct State StateType;
typedef StateType *StatePtr;
StateType fsm[4] = {
  {&forward, fsmDelay, {B, B, R, L, B, B, R, F}}, // Center
  {&left, fsmDelay, {B, B, L, F, B, B, R, F}},    // off to the Left
  {&right, fsmDelay, {B, B, R, L, B, B, F, F}},   // off to the Right state, we need to turn left
  // {&backward, random(fsmDelay * 3, fsmDelay * 4), {B, B, L, L, B, B, R, R}}
  // {&stop, fsmDelay, {S,S,S,S,S,S,S,S}}
  {&backward, 60, {B, B, L, L, B, B, R, R}}
};

uint8_t irSensorInput()
{
  uint8_t irL0, irL1, irR0, center;
  int irR1;
  center = (dSensor.getDistance()) < maxDist2Wall ? 0 : 1; // distance to center < 8cm
  irL0 = digitalRead(irLeft);
  irL1 = digitalRead(LED_BUILTIN) && irL0;

  irR1 = analogRead(A0);
  // in fact it outputs 1024 when no obstacle ahead
  irR0 = (irR1 < 1000) ? 0 : 1;
  irR1 = digitalRead(irRight) && irR0;
  // log("irL1: %u\n", irL1);
  return irL1 << 2 | center << 1 | irR1;
  //return digitalRead(irLeft) << 2 | center << 1 | digitalRead(irRight);
}

// Functions
void callback(char *topic, byte *payload, unsigned int length);

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "roachClient-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      // client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("moveCar");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  const char* mqtt_server = "ajoan.com";
  const int mqtt_port = 1883;

  Serial.begin(115200); // set up Serial library at 115200 bps
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset saved settings
  // wifiManager.resetSettings();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(180);

  //or use this for auto generated name ESP + ChipID
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect())
  {
    log("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
  //if you get here you have connected to the WiFi
  log("connected...yeey :)");

  // auto mode
  pinMode(irLeft, INPUT);
  // LED_BULLTIN is D0
  pinMode(LED_BUILTIN, INPUT);
  pinMode(irRight, INPUT);

  // Set callback
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  //log("ip addr: %u\n", WiFi.localIP());

  s = F; // state initialization
  startTime = millis();
}

void loop()
{
  uint8_t input;
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;

  if (!client.connected()) {
    reconnect();
  }

  if (selfDriving == true)
  {
    (fsm[s].cmdPtr)();
    input = irSensorInput(); // read sensors
    log("input: %u\n", input);
    if (elapsedTime >= fsm[s].delay)
    {
      // delay(fsm[s].delay);     // wait
      s = fsm[s].next[input]; // next state depends on input and current state
      log("next state: %u\n", s);
      startTime = currentTime;
    }
  }
  client.loop();
}

// Handles message arrived on subscribed topic(s)
void callback(char *topic, byte *payload, unsigned int length)
{
  log("Message arrived [");
  log(topic);
  log("] ");
  for (int i = 0; i < length; i++) {
    log((char)payload[i]);
  }
  // Handle
  moveCar(payload, length);
}

// Functions to control the robot
// int forward(String command) {
void forward()
{
  // log("motorSpeed: %u\n", motorSpeed);
  log("forward");
  rightMotor.setSpeed(motorSpeed);
  leftMotor.setSpeed(motorSpeed + 10);
}
void backward()
{
  log("backward");
  rightMotor.setSpeed(-motorSpeed);
  leftMotor.setSpeed(-motorSpeed);
}
// coz i'm lazy to change wiring
void right()
{
  log("left");
  rightMotor.setSpeed(motorSpeed);
  leftMotor.setSpeed(-motorSpeed);
  //leftMotor.setSpeed(-FIX_SPEED);
}
void left()
{
  log("right");
  rightMotor.setSpeed(-motorSpeed);
  leftMotor.setSpeed(motorSpeed);
  // rightMotor.setSpeed(MIN_SPEED);
}
void stop()
{
  // log(command);
  rightMotor.setSpeed(0);
  leftMotor.setSpeed(0);
}

void moveCar(byte * payload, unsigned int length)
{
  int params[length], i = 0, from = 0, idx = 0;
  int direction = 0;
  // convert byte (unsidned char) to String
  String command((const __FlashStringHelper*) payload);
  // 3 input params
  Serial.println("Robot params");
  idx = command.indexOf(",");
  for (i = 0; i < length; i++)
  {
    params[i] = command.substring(from, idx).toInt();
    from = idx + 1;
    idx = command.indexOf(",", from);
  }

  motorSpeed = params[0];
  maxDist2Wall = params[1];
  // fsmDelay = params[2];
  direction = params[2];
  selfDriving = params[3];
  if (selfDriving == false)
  {
    log("dir: %u\n", direction);
    switch (direction)
    {
      case 0:
        stop();
        break;
      case 1:
        forward();
        break;
      case 3:
        right();
        break;
      case 2:
        left();
        break;
      case 4:
        backward();
        break;
      default:
        stop();
        break;
    }
  }

  log("speed: %u\n", motorSpeed);
  log("dist: %u\n", maxDist2Wall);
  log("delay: %u\n", fsmDelay);
}
