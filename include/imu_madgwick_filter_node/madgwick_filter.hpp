#ifndef _MADGWICK_FILTER_H_
#define _MADGWICK_FILTER_H_

#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/math/constants/constants.hpp>

const double pi		= boost::math::constants::pi<double>();
const double eps	= 1e-8;

class MadgwickFilter
{
	public:
		MadgwickFilter(double gyro_error);
		~MadgwickFilter();
		void estimate_pose(Eigen::Vector3d acc, Eigen::Vector3d gyro);
		void reset();
		Eigen::Vector3d get_euler(){return euler;}
		Eigen::VectorXd get_quaternion(){return quaternion;};
	private:
		double beta;
		std::chrono::system_clock::time_point old_time_stamp;
		Eigen::Vector3d euler;
		Eigen::Vector4d quaternion;
};

#endif