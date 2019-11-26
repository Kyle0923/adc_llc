//https://blog.csdn.net/scliu12345/article/details/45741893

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include "adc_llc.hpp"

#define TF_PUBLISHER true

bool resetFlag = false;
WheelSpeedController* pgAdcController = nullptr;

void callback(const geometry_msgs::Twist& cmd_vel)
{

    if (-1 == cmd_vel.linear.x && -1 == cmd_vel.linear.y && -1 == cmd_vel.linear.z && \
        -1 == cmd_vel.angular.x && -1 == cmd_vel.angular.y && -1 == cmd_vel.angular.z)
    {
        // special message, reset base_link to origin
        ROS_INFO("Received a /cmd_vel message with all value == -1!, resetting base_link tf");
        resetFlag = true;
        return;
    }
    ROS_INFO("Received a /cmd_vel message!, v=[%f], w=[%f]", cmd_vel.linear.x, cmd_vel.angular.z);
    pgAdcController->setWheelSpeed(cmd_vel.linear.x, cmd_vel.angular.z);
}

// static void updateCoord(double& x, double& y, double& theta, const double linearDisplacement, const double angularDisplacement);
// static void resetCoord(double& x, double& y, double& theta);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "adc_llc");
    ros::NodeHandle rosNode;

    std::cout << "Initializing adc_llc node\n>> starting pigpio daemon..." << std::endl;

    // system("sudo pigpiod");
    if (0 != system("if ! pgrep -x \"pigpiod\" > /dev/null\nthen\nsudo pigpiod\nfi"))
    {
        throw std::runtime_error("pigpiod daemon failed to start.");
    }
    ros::Duration(0.1).sleep();

    WheelSpeedController adcController;
    pgAdcController = &adcController;

    ros::Subscriber sub = rosNode.subscribe("/cmd_vel", 1000, callback);

#if TF_PUBLISHER

    // ros::Publisher odom_pub = rosNode.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    // coordinates
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    // time util
    ros::Time current_time;

    ros::Rate loop_rate(100);

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
        if (!resetFlag)
        {
            RobotDisplacement robotDisplacement = adcController.getRobotDisplacement();
            x += robotDisplacement.linear * std::cos(theta);
            y += robotDisplacement.linear * std::sin(theta);
            theta += robotDisplacement.angular;
            // updateCoord(x, y, theta, robotDisplacement.linear, robotDisplacement.angular);
        }
        else
        {
            x = 0;
            y = 0;
            theta = 0;
            resetFlag = false;
        }

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

        // //next, we'll publish the odometry message over ROS
        // nav_msgs::Odometry odom;
        // odom.header.stamp = current_time;
        // odom.header.frame_id = "odom";

        // //set the position
        // odom.pose.pose.position.x = x;
        // odom.pose.pose.position.y = y;
        // odom.pose.pose.position.z = 0.0;
        // odom.pose.pose.orientation = odom_quat;

        // //set the velocity
        // odom.child_frame_id = "base_link";
        // odom.twist.twist.linear.x = robotSpeeds.v;
        // odom.twist.twist.angular.z = robotSpeeds.w;

        // //publish the message
        // odom_pub.publish(odom);

        loop_rate.sleep();
    }

#else
    ros::spin();
#endif

    return 0;
}

// static void updateCoord(double& x, double& y, double& theta, const double linearDisplacement, const double angularDisplacement)
// {
//     x += linearDisplacement * std::cos(theta);
//     y += linearDisplacement * std::sin(theta);
//     theta += angularDisplacement;
// }

// static void resetCoord(double& x, double& y, double& theta)
// {
//     x = 0;
//     y = 0;
//     theta = 0;
//     resetFlag = false;
// }