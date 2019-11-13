//https://blog.csdn.net/scliu12345/article/details/45741893

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include "adc_llc.hpp"

RobotSpeeds robotSpeeds {0, 0};

void callback(const geometry_msgs::Twist& cmd_vel)
{
    static WheelSpeedController controller;
    ROS_INFO("Received a /cmd_vel message!, v=[%f], w=[%f]", cmd_vel.linear.x, cmd_vel.angular.z);

    robotSpeeds = controller.setWheelSpeed(cmd_vel.linear.x, cmd_vel.angular.z);
}

static void updateCoord(double& x, double& y, double& theta, const double v, const double w, const double dt);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "adc_llc");

    std::cout << "Initializing adc_llc node\n>> starting pigpio daemon..." << std::endl;

    // system("sudo pigpiod");
    if (0 != system("if ! pgrep -x \"pigpiod\" > /dev/null\nthen\nsudo pigpiod\nfi"))
    {
        throw std::runtime_error("pigpiod daemon failed to start.");
    }

    ros::NodeHandle rosNode;
    ros::Subscriber sub = rosNode.subscribe("/cmd_vel", 1000, callback);

    ros::Publisher odom_pub = rosNode.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    // coordinates
    double x = 0;
    double y = 0;
    double theta = 0;

    // time util
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();

        // update x, y, theta
        /* just example
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;
        */
        double dt = (current_time - last_time).toSec();
        updateCoord(x, y, theta, robotSpeeds.v, robotSpeeds.w, dt);

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

#if 0
        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = robotSpeeds.v;
        odom.twist.twist.angular.z = robotSpeeds.w;

        //publish the message
        odom_pub.publish(odom);
#endif
        last_time = current_time;

        loop_rate.sleep();
    }

    return 1;
}

static void updateCoord(double& x, double& y, double& theta, const double v, const double w, const double dt)
{
    const double VelWorldX = v * std::cos(theta);
    const double VelWorldY = v * std::sin(theta);
    x += VelWorldX * dt;
    y += VelWorldY * dt;
    theta += w * dt;
}