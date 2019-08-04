#include <imu_madgwick_filter_node/imu_madgwick_filter.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imu_madgwick_filter_node");
    ros::NodeHandle nh;
    ImuMadgwickFilter imu_madgwick_filter(nh);
    ros::spin();
    return 0;
}