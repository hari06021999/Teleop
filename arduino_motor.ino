#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#define THROTTLE_PIN   2
#define REVERSE_PIN  4
#define THROTTLE_MIN_PWM  0
#define THROTTLE_MAX_PWM  200
#define FRONT_LIGHT_WHITE 6
#define BACK_LIGHT_RED 7
void onTwist(const geometry_msgs::Twist &msg);
void onLight(const std_msgs::Bool &msg);
void Forward_rightMotor(int Throttle_Pin , int Throttle_Min_Readings, int Throttle_Max_Readings, int Throttle_Min_PWM ,int Throttle_Max_PWM );
void Forward_leftMotor(int Throttle_Pin , int Throttle_Min_Readings, int Throttle_Max_Readings, int Throttle_Min_PWM ,int Throttle_Max_PWM );

ros::NodeHandle node;

ros::Subscriber<geometry_msgs::Twist> sub("/linear/angular", &onTwist);
ros::Subscriber<std_msgs::Bool> sub_led("/light", &onLight);

bool _connected = false;

void setup() {
 
    pinMode(THROTTLE_PIN, OUTPUT);
    pinMode(REVERSE_PIN,OUTPUT);
    pinMode(FRONT_LIGHT_WHITE,OUTPUT);
    digitalWrite(FRONT_LIGHT_WHITE,LOW);
    digitalWrite(REVERSE_PIN,LOW);
    node.initNode();
    node.subscribe(sub_led);
    node.subscribe(sub);
}

void stop()
{
  analogWrite(THROTTLE_PIN,0);
}
void loop() {
if (!rosConnected())
    stop();
  node.spinOnce();

}
void onLight(const std_msgs::Bool &msg)
{
   int state = msg.data;
   if(state==1)
   {
     digitalWrite(FRONT_LIGHT_WHITE,LOW);
   }
   if(state==0)
   {
     digitalWrite(FRONT_LIGHT_WHITE,HIGH);
   }
}
void onTwist(const geometry_msgs::Twist &msg)
{
  if (!_connected)
  {
    stop();
    return;
  }
  
  // Cap values at [-1 .. 1]
  float x = max(min(msg.linear.x, 1.0f), -1.0f);
  float z = max(min(msg.angular.z, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
    float l = (msg.linear.x - msg.angular.z) / 2;
    float r = (msg.linear.x + msg.angular.z) / 2;

    uint16_t lPwm = mapPwm(fabs(l), THROTTLE_MIN_PWM, THROTTLE_MAX_PWM);
    uint16_t rPwm = mapPwm(fabs(r), THROTTLE_MIN_PWM, THROTTLE_MAX_PWM);

    Forward_rightMotor(THROTTLE_PIN,0,lPwm,THROTTLE_MIN_PWM,THROTTLE_MAX_PWM);
    Forward_leftMotor(THROTTLE_PIN,0,rPwm,THROTTLE_MIN_PWM,THROTTLE_MAX_PWM);

}

void Forward_rightMotor( int Throttle_Pin  , int Throttle_Min_Readings, int Throttle_Max_Readings, int Throttle_Min_PWM ,int Throttle_Max_PWM )
{
    int Throttle_PWM_Value = map(100,Throttle_Min_Readings,Throttle_Max_Readings,Throttle_Min_PWM,Throttle_Max_PWM);
    digitalWrite(REVERSE_PIN,LOW);
    analogWrite(Throttle_Pin, Throttle_PWM_Value);

}
void Forward_leftMotor( int Throttle_Pin  , int Throttle_Min_Readings, int Throttle_Max_Readings, int Throttle_Min_PWM ,int Throttle_Max_PWM )
{
    int Throttle_PWM_Value = map(100,Throttle_Min_Readings,Throttle_Max_Readings,Throttle_Min_PWM,Throttle_Max_PWM);
    digitalWrite(REVERSE_PIN,LOW);
    analogWrite(Throttle_Pin, Throttle_PWM_Value);

}
bool rosConnected()
{
  // If value changes, notify via LED and console.
  bool connected = node.connected();
  if (_connected != connected)
  {
    _connected = connected;
  }
  return connected;
}

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}
