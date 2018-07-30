// Senior Design 2 2018
// Group 1

// ==========================
//	Lidar Node
// ==========================

#include <ros.h>
#include <sensor_msgs/LaserScan.h>
//#include <std_msgs/Bool.h>
#include <SoftwareSerial.h>
#include <TFMini.h>

// ROS Variables
ros::NodeHandle nh;
sensor_msgs::LaserScan range_msg;
ros::Publisher pub_range("range_data", &range_msg);
char frameid[] = "/tf_lidar";

// Setup software serial port
SoftwareSerial mySerial(5, 6);      // Uno RX (TFMINI TX), Uno TX (TFMINI RX)
TFMini tfmini;

// Subcriber to buggy
// Prevents buffer overrun by providing data only when buggy asks for it
/*
void messageCb(const std_msgs::Bool& lidar_msg)
{
  if(lidar_msg.data)
  {
    pub_range.publish(&range_msg);
  }
}
*/

//ros::Subscriber<std_msgs::Bool> sub_lidar("toggle_lidar", &messageCb );

void setup()
{
  Serial.begin(57600);
	// Initialize the data rate for the SoftwareSerial port
	mySerial.begin(TFMINI_BAUDRATE);

	// Initialize the TF Mini sensor
	tfmini.begin(&mySerial);

	// Initialize ROS node
	nh.initNode();
	nh.advertise(pub_range);
  //nh.subscribe(sub_lidar);

	range_msg.angle_min =       0;
	range_msg.angle_max =       3.14159;
	range_msg.angle_increment = 0.0698132;

	range_msg.time_increment =  0.00666667;
	range_msg.scan_time  =      0.4;

	range_msg.range_min =       0.3;
	range_msg.range_max =       12;
}

void loop()
{
  // TF Mini variables
  uint16_t dist = 0;
  uint16_t strength = 0;
  float range[1];
  float intensity[1];

  // Get measurement
  dist = tfmini.getDistance();
  strength = tfmini.getRecentSignalStrength();

  // publish the range value
  range[0] = {(float) dist / 100};
  // Serial.println(range[0]);
  intensity[0] = {(float) strength};

  range_msg.ranges_length = 1;
  range_msg.intensities_length = 1;
  range_msg.ranges = range;
  range_msg.intensities = intensity;

  range_msg.header.stamp = nh.now();
  pub_range.publish(&range_msg);
  
  nh.spinOnce();

  delay(5);

	// Display the measurement
	// Serial.println(range[0]);
	//Serial.print(" cm      sigstr: ");
	//Serial.println(strength);
}
