#include <imu_madgwick_filter_node/madgwick_filter.hpp>

MadgwickFilter::MadgwickFilter(double gyro_error)
{
	beta = std::sqrt(3/4) * (pi * (gyro_error/180.0));

	euler = Eigen::Vector3d::Zero();
	quaternion = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
}

MadgwickFilter::~MadgwickFilter()
{

}

void MadgwickFilter::reset()
{
	euler = Eigen::Vector3d::Zero();
	quaternion = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
}

void MadgwickFilter::estimate_pose(Eigen::Vector3d acc, Eigen::Vector3d gyro)
{
	try
	{
		std::chrono::system_clock::time_point time_stamp = std::chrono::system_clock::now(); // get time stamp

		Eigen::Vector4d d_quaternion;
		d_quaternion[0] = 0.5*(-quaternion[1]*gyro.x()-quaternion[2]*gyro.y()-quaternion[3]*gyro.z());
		d_quaternion[1] = 0.5*( quaternion[0]*gyro.x()+quaternion[2]*gyro.z()+quaternion[3]*gyro.y());
		d_quaternion[2] = 0.5*( quaternion[0]*gyro.y()-quaternion[1]*gyro.z()+quaternion[3]*gyro.x());
		d_quaternion[3] = 0.5*( quaternion[0]*gyro.z()+quaternion[1]*gyro.y()-quaternion[2]*gyro.x());

		acc = acc.normalized();

		Eigen::MatrixXd J; J.resize(3,4);
		J << -2*quaternion[2], 2*quaternion[3], -2*quaternion[0], 2*quaternion[1],
				  2*quaternion[1], 2*quaternion[0],  2*quaternion[3], 2*quaternion[2],
					0, -4*quaternion[1], -4*quaternion[2], 0;

		Eigen::Vector3d f;
		f.x() = 2*(quaternion[1]*quaternion[3] - quaternion[0]*quaternion[2]) - acc.x();
		f.y() = 2*(quaternion[0]*quaternion[1] - quaternion[2]*quaternion[3]) - acc.y();
		f.z() = 2*(0.5 - std::pow(quaternion[1], 2) - std::pow(quaternion[2], 2)) - acc.z();

		Eigen::VectorXd f_nable; f_nable.resize(4,1);
		f_nable = J.transpose() * f;
		f_nable = f_nable.normalized();

		double delta = std::chrono::duration_cast<std::chrono::milliseconds>(time_stamp-old_time_stamp).count() / 1000.0;
		quaternion += (d_quaternion-(beta*f_nable)) * delta;
		quaternion = quaternion.normalized();

		euler.x() = std::atan2(2*(quaternion[0]*quaternion[1] + quaternion[2]*quaternion[3]), -2*(std::pow(quaternion[1], 2)) + std::pow(quaternion[2], 2) + 1);
		euler.y() = -std::asin(-2*(quaternion[1]*quaternion[3] + quaternion[0]*quaternion[2]));
		euler.z() = std::atan2(2*(quaternion[0]*quaternion[3] + quaternion[1]*quaternion[2]), -2*(std::pow(quaternion[2], 2)) + std::pow(quaternion[0], 2) + 1);

		old_time_stamp = time_stamp;
	}
	catch(std::exception& ex)
	{
		std::cerr << ex.what() << std::endl;
	}
}
