#ifndef _IMU_MADGWICK_FILTER_H_
#define _IMU_MADGWICK_FILTER_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <imu_madgwick_filter_node/madgwick_filter.hpp>

class ImuMadgwickFilter
{
    public:
        ImuMadgwickFilter(ros::NodeHandle nh);
        ~ImuMadgwickFilter(){}
    private:
        void _imu_call_back(const sensor_msgs::ImuConstPtr &msg);

        ros::NodeHandle _nh;
        ros::Subscriber _imu_in;
        ros::Publisher _imu_out;

        MadgwickFilter *_madgwick;
};

#endif