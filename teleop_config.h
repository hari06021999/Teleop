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
const uint8_t L_PWM = 8;//D8
const uint8_t front_light = 5;
const uint8_t front_light_white = 6;
const uint8_t front_light_yellow = 7;
const uint8_t brake_light = 1;
const uint8_t forward_brake = 10;
const uint8_t reverse_brake = 11;

bool _connected = false;

enum {ZERO,ONE,TWO,THREE,FOUR,FIVE};
