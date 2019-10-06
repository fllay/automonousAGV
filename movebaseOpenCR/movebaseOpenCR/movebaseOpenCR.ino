#include "ModbusBLVmotor.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "lino_velocities.h"
#include "geometry_msgs/Twist.h"
#include "lino_imu.h"
#include "Kinematics.h"

#include <IMU.h>

#define LINO_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors
#define MAX_RPM 100               // motor's maximum RPM
#define WHEEL_DIAMETER 0.043       // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.10800  // distance between left and right wheels 0.800
#define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN

#define COMMAND_RATE 20

cIMU    IMU;

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);


float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;
unsigned long g_prev_command_time = 0;

int currentLeftWheelRPM;
int currentRightWheelRPM;

void commandCallback(const geometry_msgs::Twist& cmd_msg);
void moveBase();


void waitForSerialLink(bool isConnected);


ros::NodeHandle nh;


geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;

ModbusBLVmotor md; 


ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);

lino_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

int led_pin = 13;
unsigned long prev_update_time_m1 = 0;
unsigned long prev_update_time_m2 = 0;

unsigned long pulseRPM1 = 0;
unsigned long pulseRPM2 = 0;

unsigned long prev_update_time_M1 = 0;
unsigned long prev_update_time_M2 = 0;

unsigned long prev_encoder_ticks_M1 = 0;
unsigned long prev_encoder_ticks_M2 = 0;
int counts_per_rev_ = 30;


void m1Pulse(void){
  unsigned long current_time_m1 = micros();;
  unsigned long dt = current_time_m1 - prev_update_time_m1;
  prev_update_time_m1 = micros();
  float freq_m1 = 1/(dt*0.000001);
  pulseRPM1++;
  //Serial.print("m1 = ");
  //Serial.println(freq_m1);
}

void m2Pulse(void){
  unsigned long current_time_m2 = micros();;
  unsigned long dt = current_time_m2 - prev_update_time_m2;
  prev_update_time_m2 = micros();
  float freq_m2 = 1/(dt*0.000001);
  pulseRPM2++;
  //Serial.print("m2 = ");
  //Serial.println(freq_m2);  
}


int getRPM1(){
    unsigned long encoder_ticks = pulseRPM1;
    //this function calculates the motor's RPM based on encoder ticks and delta time
    unsigned long current_time = millis();
    unsigned long dt = current_time - prev_update_time_M1;

    //convert the time from milliseconds to minutes
    double dtm = (double)dt / 60000.0;
    double delta_ticks = encoder_ticks - prev_encoder_ticks_M1;

    //calculate wheel's speed (RPM)

    prev_update_time_M1 = current_time;
    prev_encoder_ticks_M1 = encoder_ticks;
    
    return (delta_ticks / counts_per_rev_) / dtm; 
}

int getRPM2(){
    unsigned long encoder_ticks = pulseRPM2;
    //this function calculates the motor's RPM based on encoder ticks and delta time
    unsigned long current_time = millis();
    unsigned long dt = current_time - prev_update_time_M2;

    //convert the time from milliseconds to minutes
    double dtm = (double)dt / 60000.0;
    double delta_ticks = encoder_ticks - prev_encoder_ticks_M2;

    //calculate wheel's speed (RPM)

    prev_update_time_M2 = current_time;
    prev_encoder_ticks_M2 = encoder_ticks;
    
    return (delta_ticks / counts_per_rev_) / dtm;    
    
}

void setLeftRPM(int rpm){
    if(rpm < 0){
        //M1_dir = 1;
        currentLeftWheelRPM = -1*getRPM1();
    } else {
        //M1_dir = 0;
        currentLeftWheelRPM = getRPM1();
    }
    
        

   md.setM1Speed(rpm);     
       
}

void setRightRPM(int rpm){
    
       if(rpm < 0){
        //M2_dir = 1;
        currentRightWheelRPM = -1*getRPM2();
    } else {
        //M2_dir = 0;
        currentRightWheelRPM = getRPM2();
    }

    
        

    md.setM2Speed(rpm);
}


void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(cmd_sub);
  nh.advertise(raw_vel_pub);
  nh.advertise(raw_imu_pub);


  
  
  //Serial.begin(115200); 
  //Serial.println("Start");
  pinMode(7, INPUT); //set Arduino Pin 2 as input with pull-down
  attachInterrupt(3, m1Pulse, RISING);


  pinMode(8, INPUT); //set Arduino Pin 2 as input with pull-down
  attachInterrupt(4, m2Pulse, RISING);
  
  pinMode(led_pin, OUTPUT);
  md.init();  
  IMU.begin();


}

void loop() {
  
  static unsigned long prev_imu_time = 0;
  static unsigned long prev_control_time = 0;
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
  

  //Serial.println("5555");
  //Serial2.print("66656");


        //this block drives the robot based on defined rate
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE)){   
    moveBase();
    //sprintf (buffer, "Encoder FrontLeft  : %i Encoder FrontRight  : %i ", currentLeftWheelRPM, currentRightWheelRPM);
    //nh.loginfo(buffer);
    prev_control_time = millis();
  }
  if ((millis() - prev_imu_time) >= 50)
        {

          IMU.update();
                   
            //imu.setmode(OPERATION_MODE_NDOF_FMC_OFF);
    
            raw_imu_msg.linear_acceleration.x = IMU.accRaw[0];
            raw_imu_msg.linear_acceleration.y = IMU.accRaw[1];
            raw_imu_msg.linear_acceleration.z = IMU.accRaw[2];

           
            raw_imu_msg.angular_velocity.x = IMU.gyroRaw[0];
            raw_imu_msg.angular_velocity.y = IMU.gyroRaw[1];
            raw_imu_msg.angular_velocity.z = IMU.gyroRaw[2];

            
            raw_imu_msg.magnetic_field.x = IMU.magRaw[0];
            raw_imu_msg.magnetic_field.y = IMU.magRaw[1];
            raw_imu_msg.magnetic_field.z = IMU.magRaw[2];


            raw_imu_pub.publish(&raw_imu_msg);
           


           prev_imu_time = millis();
        }

  nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
}

void stopBase()
{   
    
  md.setM1Speed(0);
  //delay(2000);  
  md.setM2Speed(0);
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

void moveBase(){
    
    Kinematics::velocities current_vel;
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
    setLeftRPM(req_rpm.motor1);
    setRightRPM(req_rpm.motor2);


      
    int current_rpm1 = currentRightWheelRPM; //rightWheel.getRPM();
    int current_rpm2 = currentLeftWheelRPM; //leftWheel.getRPM();
    int current_rpm3 = 0;
    int current_rpm4 = 0;


    current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    
    
    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);
}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  
  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;


    //moveBase();


    //sprintf (buffer, "Encoder FrontLeft  : %i Encoder FrontRight  : %i ", currentLeftWheelRPM, currentRightWheelRPM);
    //nh.loginfo(buffer);
            
    g_prev_command_time = millis();
}
