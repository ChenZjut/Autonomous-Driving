/**
 * 该例程将发布chatter话题，消息类型String
 */
 
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <can_msgs/ecu.h>
const double PI = 3.14159265358979f;
ros::Publisher ecu_pub;
uint8_t motor_shift;
void upper_vel(const geometry_msgs::TwistConstPtr &msg)
{
    can_msgs::ecu ecu_tmp_msg;
    double motor_vel;// Ackermann_msg下的线速度
    double angle_vel;
    motor_vel = (msg->linear.x) * 3.6;  //线速度 m/s，接收到的信息
    angle_vel = (msg->angular.z) * 120 / 27 / (PI / 180);  //转向角度 单位是弧度
    if (motor_vel > 0.05)
   {
	motor_shift = 1;
      
   }
    else if(motor_vel < -0.05)
   {
        motor_shift = 3;
   }
    else if(motor_vel <= 0.05 && motor_vel >= -0.05 && angle_vel <= 3 && angle_vel >= -3)
   {
        motor_shift = 2;
   }

   if(angle_vel>1200)
   {
      angle_vel = 1200;
   }
   else if(angle_vel <-1200)
   {
      angle_vel = -1200;
   }
    ecu_tmp_msg.motor = motor_vel;
    ecu_tmp_msg.steer = angle_vel;
    ecu_tmp_msg.shift = motor_shift;
    ecu_tmp_msg.brake = 0;
    ecu_pub.publish(ecu_tmp_msg);
}

int main(int argc, char **argv)
{
  // ROS节点初始化
  ros::init(argc, argv, "twist_to_ecu");
  ros::NodeHandle n;
  
  // 创建一个Publisher，发布名为chatter的topic，消息类型为std_msgs::String
  ecu_pub = n.advertise<can_msgs::ecu>("ecu", 10, true);
  ros::Subscriber upper_vel_sub = n.subscribe("cmd_vel",10, &upper_vel);//订阅速度消息
  ros::spin();
  // 设置循环的频率
  /*ros::Rate loop_rate(10);

  while (ros::ok())
  {
	// 初始化std_msgs::String类型的消息
	// 循环等待回调函数
    ros::spinOnce();	
	// 按照循环频率延时
    loop_rate.sleep();

  }
*/
  return 0;
}
