

#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <SoftwareSerial.h>
SoftwareSerial portOne(11, 12);
// The min amount of PWM the motors need to move. Depends on the battery, motors and controller.
// The max amount is defined by PWMRANGE in Arduino.h
#define PWM_MIN -255
#define PWMRANGE 255

// Declare functions
void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
void onTwist(const geometry_msgs::Twist &msg);
float mapPwm(float x, float out_min, float out_max);

// Pins
const uint8_t R_PWM = 5;
const uint8_t R_BACK = 6;
const uint8_t R_FORW = 7;
const uint8_t L_BACK = 8;
const uint8_t L_FORW = 9;
const uint8_t L_PWM = 10;


// ROS serial server

ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);

bool _connected = false;

void setup()
{
  setupPins();
  portOne.begin(9600);
  portOne.println("hello Wlcome");
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  node.initNode();
  node.subscribe(sub);
  
}

void setupPins()
{
  

  pinMode(L_PWM, OUTPUT);
  pinMode(L_FORW, OUTPUT);
  pinMode(L_BACK, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(R_FORW, OUTPUT);
  pinMode(R_BACK, OUTPUT);
  stop();
  
}



void stop()
{
  digitalWrite(L_FORW, 0);
  digitalWrite(L_BACK, 0);
  digitalWrite(R_FORW, 0);
  digitalWrite(R_BACK, 0);
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}

void onTwist(const geometry_msgs::Twist &msg)
{
  if (!_connected)
  {
    stop();
    return;
  }
   digitalWrite(13,HIGH);
   digitalWrite(13,LOW);
  portOne.print("Linear:");
  portOne.println(msg.linear.x);
  portOne.print("Angular:");
  portOne.println(msg.angular.z);
  // Cap values at [-1 .. 1]
  float x = max(min(msg.linear.x, 1.0f), -1.0f);
  float z = max(min(msg.angular.z, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
  uint16_t lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
  uint16_t rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);
  digitalWrite(13,HIGH);
   digitalWrite(13,LOW);
  // Set direction pins and PWM
  digitalWrite(L_FORW, l > 0);
  digitalWrite(L_BACK, l < 0);
  digitalWrite(R_FORW, r > 0);
  digitalWrite(R_BACK, r < 0);
  analogWrite(L_PWM, lPwm);
  analogWrite(R_PWM, rPwm);
  portOne.print("lpwm:");
  portOne.println(lPwm);
  portOne.print("rpwm:");
  portOne.println(rPwm);
}

void loop()
{
  if (!rosConnected())
    stop();
  node.spinOnce();
}

bool rosConnected()
{
  // If value changes, notify via LED and console.
  bool connected = node.connected();
  if (_connected != connected)
  {
    _connected = connected;
  
    portOne.println(connected ? "ROS connected" : "ROS disconnected");
  }
  return connected;
}

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}
