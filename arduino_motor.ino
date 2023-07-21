

#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <PWM.h>
//#include <SoftwareSerial.h>
//SoftwareSerial portOne(11, 12);
// The min amount of PWM the motors need to move. Depends on the battery, motors and controller.
// The max amount is defined by PWMRANGE in Arduino.h
#define PWM_MIN 80
#define PWMRANGE 120
int frequency_right = 350; //frequency (in Hz)
int frequency_left = 400; //frequency (in Hz)

// Declare functions
void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
void onTwist(const geometry_msgs::Twist &msg);
void onfront(const std_msgs::Bool &msg);
void onfont_multi(const std_msgs::Int16 &msg);
float mapPwm(float x, float out_min, float out_max);
void brake(void);

// Pins
const uint8_t R_PWM = 9;//D3
const uint8_t right_relay = 2;// D2 RIGHT 1ST MOTOR
const uint8_t left_relay = 4;// D4 RIGHT 2ND MOTOR
const uint8_t L_PWM = 3;//D9
const uint8_t front_light = 5;
const uint8_t front_light_white = 6;
const uint8_t front_light_yellow = 7;
const uint8_t brake_light = 1;


// ROS serial server

ros::NodeHandle node;
ros::Subscriber<std_msgs::Bool> sub_light1("front/light",&onfront);
ros::Subscriber<std_msgs::Int16> sub_light2("front/light/multi",&onfont_multi);
ros::Subscriber<geometry_msgs::Twist> sub("/linear/angular", &onTwist);

bool _connected = false;

void setup()
{
  InitTimersSafe(); 
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

  InitTimersSafe();
  bool success = SetPinFrequencySafe(L_PWM, frequency_left);
  SetPinFrequencySafe(R_PWM, frequency_right);
  
  setupPins();
  node.initNode();
  node.subscribe(sub);
  node.subscribe(sub_light1);
  node.subscribe(sub_light2);
  
}

void setupPins()
{
   
  pinMode(brake_light,OUTPUT);
  pinMode(right_relay, OUTPUT);
  pinMode(left_relay, OUTPUT);
   pinMode(front_light, OUTPUT);
  pinMode(front_light_white, OUTPUT);
  pinMode(front_light_yellow, OUTPUT);
  stop();
  
}



void stop()
{
    digitalWrite(brake_light, HIGH);
  digitalWrite(right_relay, HIGH);
  digitalWrite(left_relay, HIGH);
  digitalWrite(front_light, HIGH);
  digitalWrite(front_light_white, HIGH); 
  digitalWrite(front_light_yellow, HIGH);
    pwmWrite(L_PWM, 0);
  pwmWrite(R_PWM, 0);
  
}

void onTwist(const geometry_msgs::Twist &msg)
{

  if (!_connected)
  {
    stop();
    return;
  }
if(msg.linear.x>0||msg.linear.x<0)
{
  // Cap values at [-1 .. 1]
  float x = max(min(msg.linear.x, 1.0f), -1.0f);
  float z = max(min(msg.angular.z, 1.0f), -1.0f);
  if(x==0 && z==0)
  {
    brake();
  }
  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
  uint16_t lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
  uint16_t rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);

  // Set direction pins and PWM
       digitalWrite(left_relay, l>0);
       digitalWrite(right_relay, r>0);

  digitalWrite(LED_BUILTIN, HIGH);
  for(int i=25,j=25;i<lPwm,j<rPwm;i++,j++)
  {
   
   pwmWrite(L_PWM, i);
   pwmWrite(R_PWM, j);
   delay(100);
  }
}
else
{
  digitalWrite(LED_BUILTIN, LOW);
  pwmWrite(L_PWM, 0);
  pwmWrite(R_PWM, 0);
}
}
void onfront(const std_msgs::Bool &msg)
{
  int check = msg.data;
  if(check == 1)
  {
   

    digitalWrite(front_light, LOW);
   
  }
   if(check == 0)
  {
    digitalWrite(front_light, HIGH);
  
  }
}
void onfont_multi(const std_msgs::Int16 &msg)
{
  int check = msg.data;
  if(check == 1)
  {
   
    digitalWrite(front_light_white, LOW);
    digitalWrite(front_light_yellow, HIGH);

  }
   if(check == 2)
  {
    digitalWrite(front_light_yellow, LOW);
    digitalWrite(front_light_white, HIGH);
  }
  if(check == 0)
  {
    digitalWrite(front_light_white, HIGH);
    digitalWrite(front_light_yellow, HIGH);
  }
}
void brake(void)
{
 
        digitalWrite(brake_light, LOW);
//        delay(1000);
        digitalWrite(brake_light, HIGH);
  
}
void loop()
{
  if (!rosConnected())
    stop();
  node.spinOnce();
//  while(1)
//  {
//    digitalWrite(LED_BUILTIN, HIGH);
//    delay(500);
//    digitalWrite(LED_BUILTIN, LOW);
//    delay(500);
//     if (!rosConnected())
//        node.spinOnce();
//  }
}

bool rosConnected()
{
  // If value changes, notify via LED and console.
  bool connected = node.connected();
  if (_connected != connected)
  {
    _connected = connected;
  
   // portOne.println(connected ? "ROS connected" : "ROS disconnected");
  }
  return connected;
}

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}
