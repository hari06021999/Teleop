
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <DuePWM.h>
#include <stdint.h>
#include "teleop_config.h"




// ROS serial server
DuePWM pwm( frequency_right, frequency_left );
ros::NodeHandle node;
ros::Subscriber<std_msgs::Bool> sub_light1("front/light",&onfront);
ros::Subscriber<std_msgs::Int16> sub_light2("front/light/multi",&onfont_multi);
ros::Subscriber<geometry_msgs::Twist> sub("/linear/angular", &onTwist);



void setup()
{
  pwm.setFreq1( frequency_right );
  pwm.setFreq2( frequency_left );
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
  pwm.pinFreq1( L_PWM );  // Pin 7 freq set to "pwm_freq2" on clock B
  pwm.pinFreq2( R_PWM );  // Pin 8 freq set to "pwm_freq2" on clock B
  
  setupPins();
  node.initNode();
  node.subscribe(sub);
  node.subscribe(sub_light1);
  node.subscribe(sub_light2);
  
}

void setupPins()
{
   
 
  pinMode(right_relay, OUTPUT);
  pinMode(left_relay, OUTPUT);
  pinMode(front_light, OUTPUT);
  pinMode(front_light_white, OUTPUT);
  pinMode(front_light_yellow, OUTPUT);
  pinMode(forward_brake, OUTPUT);
  pinMode(reverse_brake, OUTPUT);
  pinMode(reverse_light, OUTPUT);
  stop();
  
}



void stop()
{
  digitalWrite(right_relay, HIGH);
  digitalWrite(left_relay, HIGH);
  digitalWrite(front_light, HIGH);
  digitalWrite(front_light_white, HIGH); 
  digitalWrite(front_light_yellow, HIGH);
  digitalWrite(forward_brake, HIGH); 
  digitalWrite(reverse_brake, HIGH);
  digitalWrite(reverse_light, HIGH);
  pwm.pinDuty(L_PWM, 0);
  pwm.pinDuty(R_PWM, 0);
  
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
  
  // Calculate the intensity of left and right wheels. Simple version.
  uint8_t right_flag=0,left_flag=0;
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
  uint16_t lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
  uint16_t rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);

  // Set direction pins and PWM
       digitalWrite(left_relay, l<0);
       digitalWrite(right_relay, r<0);
  if(l<0 && r<0)
  {
    right_flag=1;
    left_flag=1;
  }
  digitalWrite(LED_BUILTIN, HIGH);
  for(uint16_t i=25,j=25;i<lPwm,j<rPwm;i++,j++)
  {
   if(i<=lPwm)
		pwm.pinDuty(L_PWM, i);
   if(j<=rPwm)
		pwm.pinDuty(R_PWM, j);
   if(right_flag==1 && left_flag==1)
   {
     digitalWrite(reverse_light, LOW);
   }
   delay(100);
  }
}
else
{
  digitalWrite(LED_BUILTIN, LOW);
  pwm.stop(L_PWM);
  pwm.stop(R_PWM);
  brake();
}
}
void onfront(const std_msgs::Bool &msg)
{
  uint8_t frontlight_data = msg.data;
  if(frontlight_data == ONE)
  {
   

    digitalWrite(front_light, LOW);
   
  }
   if(frontlight_data == ZERO)
  {
    digitalWrite(front_light, HIGH);
  
  }
}
void onfont_multi(const std_msgs::Int16 &msg)
{
  uint8_t  front_mutli_light_data = msg.data;
  if(front_mutli_light_data == ONE)
  {
   
    digitalWrite(front_light_white, LOW);
    digitalWrite(front_light_yellow, HIGH);

  }
   if(front_mutli_light_data == TWO)
  {
    digitalWrite(front_light_yellow, LOW);
    digitalWrite(front_light_white, HIGH);
  }
  if(front_mutli_light_data == ZERO)
  {
    digitalWrite(front_light_white, HIGH);
    digitalWrite(front_light_yellow, HIGH);
  }
}
void brake(void)
{
      digitalWrite(reverse_light, HIGH);
      digitalWrite(forward_brake, LOW); 
      digitalWrite(reverse_brake, HIGH);
      delay(1000);
      digitalWrite(forward_brake, HIGH); 
      digitalWrite(reverse_brake, LOW);
      delay(1000);
      digitalWrite(forward_brake, HIGH); 
      digitalWrite(reverse_brake, HIGH);
  
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
  }
  return connected;
}

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}
