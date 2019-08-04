#include <imu_madgwick_filter_node/imu_madgwick_filter.hpp>

ImuMadgwickFilter::ImuMadgwickFilter(ros::NodeHandle nh)
    : _nh(nh)
{
    _imu_in = _nh.subscribe("/imu/data", 100, &ImuMadgwickFilter::_imu_call_back, this);
    _imu_out = _nh.advertise<sensor_msgs::Imu>("/imu/data_madgwick2", 100);

    _madgwick = new MadgwickFilter(0.1);
}

void ImuMadgwickFilter::_imu_call_back(const sensor_msgs::ImuConstPtr &msg)
{
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;

    acc.x() = msg->linear_acceleration.x;
    acc.y() = msg->linear_acceleration.y;
    acc.z() = msg->linear_acceleration.z;

    gyro.x() = msg->angular_velocity.x;
    gyro.y() = msg->angular_velocity.y;
    gyro.z() = msg->angular_velocity.z;

    // estimate posture
    _madgwick->estimate_pose(acc, gyro);

    Eigen::Vector3d euler = _madgwick->get_euler();
    Eigen::Vector4d quaternion = _madgwick->get_quaternion();

    sensor_msgs::Imu imu_madgwick;
    imu_madgwick.header.frame_id = msg->header.frame_id;
    imu_madgwick.header.stamp = ros::Time::now();

    imu_madgwick.linear_acceleration = msg->linear_acceleration;
    imu_madgwick.angular_velocity = msg->angular_velocity;
    
    imu_madgwick.orientation.w = quaternion[0];
    imu_madgwick.orientation.x = quaternion[1];
    imu_madgwick.orientation.y = quaternion[2];
    imu_madgwick.orientation.z = quaternion[3];

    _imu_out.publish(imu_madgwick);
}