//https://blog.csdn.net/scliu12345/article/details/45741893

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include "adc_llc.hpp"

void callback(const geometry_msgs::Twist& cmd_vel)
{
    static WheelSpeedController controller;
    static ros::NodeHandle rosNode;
    static ros::Publisher llc_pub = rosNode.advertise<geometry_msgs::Twist>("/robot_vel", 1000);
    ROS_INFO("Received a /cmd_vel message!, v=[%f], w=[%f]", cmd_vel.linear.x, cmd_vel.angular.z);

    RobotSpeeds robotSpeeds = controller.setWheelSpeed(cmd_vel.linear.x, cmd_vel.angular.z);
    geometry_msgs::Twist robotVel;
    robotVel.linear.x = robotSpeeds.v;
    robotVel.angular.z = robotSpeeds.w;
    llc_pub.publish(robotVel);
    // Do velocity processing here:
    // Use the kinematics of your robot to map linear and angular velocities into motor commands
//    v_l = ...
//    v_r = ...

    // Then set your wheel speeds (using wheel_left and wheel_right as examples)
//    wheel_left.set_speed(v_l)
//    wheel_right.set_speed(v_r)
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_listener");
    ros::NodeHandle rosNode;
    ros::Subscriber sub = rosNode.subscribe("/cmd_vel", 1000, callback);
    ros::spin();

    return 1;
}