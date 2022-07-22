/*Code for simple navigation of Pioneer 3DX robot in MobileSim simulation.
Robot smoves forward, senses two frontal sonars and turns right 90 deg if there's an obstacle.
Master's degree course of Mobile Robotis @ UFRGS, Brazil.
Author: fupbot (github)
Date: July, 2022
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <iomanip> 

//Global variables for sonar readings
float x_left = 0.0;
float y_left = 0.0;
float x_right = 0.0;
float y_right = 0.0;

//gets readings from two frontal sonars and writes them to global variables
void sonar_rec(const sensor_msgs::PointCloud& son) 
{
  x_left = son.points[3].x;
  y_left = son.points[3].y;

  x_right = son.points[4].x;
  y_right = son.points[4].y;
}

//checks pose messages and outputs them to user 
void poseMessageReceived(const nav_msgs::Odometry& msg) 
{
	std::cout << std::setprecision(2) << std::fixed << /* output the pose information using standard output */
	  "Current position=(" << msg.pose.pose.position.x << ", " << msg.pose.pose.position.y << ") " << 
	  "Current direction=" << std::setprecision(2) << std::fixed << msg.pose.pose.orientation.w<<"\r";
	std::flush(std::cout);
}


//main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_simple");
  ros::NodeHandle nh;
  
  //publishers
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1000);
  geometry_msgs::Twist msg;

  //subscribers
  ros::Subscriber pose = nh.subscribe("RosAria/pose", 1000, &poseMessageReceived);
  ros::Subscriber sub = nh.subscribe("RosAria/sonar", 1000, sonar_rec);
  sensor_msgs::PointCloud sonar;

  ROS_INFO_STREAM("AQUI");

  double BASE_SPEED = 0.2, MOVE_TIME = 3.0, CLOCK_SPEED = 0.5, PI = 3.14159;
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);

  // Make the robot stop at the beggining of simulation
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;
  pub.publish(msg);
//
//main loop
  while(ros::ok())
    {
      if (x_left > 1.0 && x_right > 1.0){        //move forward - no obstacle within 1m
        msg.linear.x = BASE_SPEED;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;
	      pub.publish(msg);
        //ROS_INFO_STREAM("Moving forward at 0.2 m/s");
      
        ros::spinOnce();
        //rate.sleep();
      } 
      else if (x_left < 1.0){   //turn right 45 deg
        msg.angular.z = - PI / 4;
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
	      pub.publish(msg);
        ROS_INFO_STREAM("Oh no, obstacle detected! Turning right!");
      
        ros::spinOnce();
        rate.sleep();
      }
      else{                                     //turn left 45 deg
        msg.angular.z = PI / 4;
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
	      pub.publish(msg);
        ROS_INFO_STREAM("Oh no, obstacle detected! Turning left!");
      
        ros::spinOnce();
        rate.sleep();
      }
   }
}
