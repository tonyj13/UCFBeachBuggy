// Senior Design 2 2018
// Group 1

// ==========================
//  Servo Node
// ==========================

#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

// ROS Variables
ros::NodeHandle nh;
std_msgs::Int16 servoAngleROS;
ros::Publisher pub_servo("servo_status", &servoAngleROS);
char servoToggle = 0;

// Servo Variables
Servo buggyServo;
char servoPin = 3;
int servoAngle;   // servo position in degrees

// Subcriber to buggy
void messageCb(const std_msgs::Int16& servoAngle_msg)
{
  servoAngle = servoAngle_msg.data;

  // move servo
  buggyServo.write(servoAngle);
  servoAngleROS.data = servoAngle;
  pub_servo.publish( &servoAngleROS );
}

ros::Subscriber<std_msgs::Int16> sub_servo("toggle_servo", &messageCb );

void setup()
{
  // Initialize hardware serial port (serial debug port)
  Serial.begin(57600);
  
  // Initialize servo and set to 0 degrees (right)
  buggyServo.attach(servoPin);
  buggyServo.write(0);

  // Initialize ROS node
  nh.initNode();
  nh.advertise(pub_servo);
  nh.subscribe(sub_servo);
}

void loop()
{
  nh.spinOnce();
}


