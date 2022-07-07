#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/PointCloud.h>
#include<iomanip> // for std :: setprecision and std :: fixed


/* checks pose messages and outputs them to user */
void sonar_rec(const sensor_msgs::PointCloud& son) 
{
	//std::cout << std::setprecision(2) << std::fixed << "Sonar: " << son.points;
  ROS_INFO_STREAM(son.points[0]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_simple");
  ros::NodeHandle nh;
  
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1000);
  geometry_msgs::Twist msg;

  ros::Subscriber sub = nh.subscribe("RosAria/sonar", 1000, sonar_rec);
  sensor_msgs::PointCloud sonar;

  double BASE_SPEED = 0.2, MOVE_TIME = 3.0, CLOCK_SPEED = 0.5,  PI = 3.14159;
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);

  //leitura do sonar
  //sonar = nh.subscribe("sensor_msgs/PointCloud", 1000, &sonar_rec);

  // Make the robot stop (robot perhaps has a speed already)
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;
  pub.publish(msg);

  //while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED + 1)
  while(ros::ok())
    {
      //leitura do sonar
      //sonar = nh.subscribe("sensor_msgs/PointCloud", 1000, &sonar_rec);
      // sonar = sub.subscribe(sonar);
      // ROS_INFO_STREAM(sonar);
	    msg.linear.x = BASE_SPEED;
      msg.angular.z = -1 * PI/ int(MOVE_TIME/CLOCK_SPEED) / 4;
	    pub.publish(msg);
      ROS_INFO_STREAM("O robo esta navegando sozinho!");
      //count++;
      ros::spinOnce();
      rate.sleep();
   }
  
  // make the robot stop
  // for (int i = 0; i < 2; i++)
  //   {  

  //     msg.linear.x = 0;
  //     msg.linear.y = 0;
  //     msg.linear.z = 0;
  //     pub.publish(msg);

  //   }
  //   ROS_INFO_STREAM("O robo parou.");
    
    // Guard, make sure the robot stops.
    // rate.sleep();
    // msg.linear.x = 0;
    // msg.linear.y = 0;
    // msg.linear.z = 0;
    // pub.publish(msg); 

}
