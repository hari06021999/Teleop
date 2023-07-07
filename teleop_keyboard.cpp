#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "mod8i8o.h"
#include "rs485_modbus_rtu.h"
#include "mod8i8o.cpp"
#include "rs485_modbus_rtu.cpp"
#include <unistd.h>
#include <stdio.h>
#include <sstream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include<stdlib.h>
#include <termios.h>
#include <ctime>
using namespace std;

const char *msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
        f    
   l    s    r
        b    

Valve open/close: o/O
Y-foward/Reverse - w/s
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
)";

char key(' ');

int getch(void)
{
   int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "teleop_twist_keyboard");
  ros::NodeHandle n;
  RS485_Modbus_RTU test1(testDevParam);
	//getting modbus pointer variable for local access
  modbus_t *ctx = test1.getDev();
     int read_setpin_status;
 int read_resetpin_status;


         MOD8I8O ioDev1;
	ioDev1.mapModbus(test1.isModbusOpen(), ctx, 1);
   printf("%s", msg);
    while (ros::ok())
  {

    // Get the pressed key
    key = getch();

    switch (key)
    {
    case 'f':
    case 'F':
      std::cout << "Up=" << key;
      ioDev1.setOutputPort(0x05);
      ioDev1.setOutputPort(0x00);
      break;
    case 'b':
    case 'B':
      std::cout << "Down=" << key;
      ioDev1.setOutputPort(0x0A);
      ioDev1.setOutputPort(0x00);
      break;

    case 'r':
    case 'R':
      std::cout << "Right=" << key;
      ioDev1.setOutputPort(0x09); 
      ioDev1.setOutputPort(0x00);
      break;
    case 'l':
    case 'L':
      std::cout << "Left=" << key;
      ioDev1.setOutputPort(0x06); 
      ioDev1.setOutputPort(0x00); 
      break;

    case 's':
    case 'S':
      std::cout << "Stop=" << key;
      ioDev1.setOutputPort(0x00);
      break;
   
    default:
      std::cout << "key unknown=" << key;
      break;
    }
    if(key=='\x03')
    break;
  }
  ros::spinOnce();
}
