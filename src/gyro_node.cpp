#include <stdexcept>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "gyro_gpio_if.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "gyro_node");
    std::cout << "Initializing gyro node\n>> starting pigpio daemon..." << std::endl;

    if (0 != system("if ! pgrep -x \"pigpiod\" > /dev/null\nthen\nsudo pigpiod\nfi"))
    {
        throw std::runtime_error("pigpiod daemon failed to start.");
    }

    tf::TransformBroadcaster odom_broadcaster;

    GyroGpioIF gyro;
    GyroData gyroData;
    std::cout << "Calibrating MPU6050, please wait for 1 sec..." << std::endl;
    gyro.calibration(1, 100);

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;

    double dt = 0.0;
    double delta_x = 0.0;
    double delta_y = 0.0;
    double delta_th = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        gyroData = gyro.readGyroData();
        ROS_INFO("ACCEL: %f, %f, %f\t\tGYRO: %f, %f, %f", gyroData.accel_x, gyroData.accel_y, gyroData.accel_z, gyroData.gyro_x, gyroData.gyro_y, gyroData.gyro_z);
        // ros::Duration(0.5).sleep();
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        dt = (current_time - last_time).toSec();

        vx += gyroData.accel_x;
        vy += gyroData.accel_y;
        vth = gyroData.gyro_z;

        delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

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

        last_time = current_time;
        loop_rate.sleep();
    }
}