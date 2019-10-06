#include "ModbusBLVmotor.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

//#include <IMU.h>


ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);


geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;

ModbusBLVmotor md; 



int led_pin = 13;


void setup() {
  // put your setup code here, to run once:
 
  pinMode(led_pin, OUTPUT);
  md.init();  


}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(led_pin, HIGH);  // set to as HIGH LED is turn-off
  delay(200);                   // Wait for 0.1 second
  digitalWrite(led_pin, LOW);   // set to as LOW LED is turn-on
  delay(200);         // Wait for 0.1 second
  md.setM1Speed(40);
  //delay(2000);  
  md.setM2Speed(40);
  //delay(2000); 
  //md.setM2Speed(0); 
  //delay(2000); 
  

  //Serial1.print("5555");
  //Serial2.print("66656");

}
