// xv11_raw2imu_node.cpp
// Takes the packets of raw imu data and compiles it into an IMU message

// Author: Ryker Dial
// Created: July 26, 2017
// Last Modified: October 20, 2017

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <vector>
#include <cmath>
#include "covariance_matrix.h" // For calculating incremental covariance matrices

#include <fstream>
#include <iostream>

//#define LOG_DATA
#ifdef LOG_DATA
	std::ofstream log_file;
	std::ofstream o_cov_file;
	std::ofstream av_cov_file;
	std::ofstream la_cov_file;
	std::string log_output_dir = "";
	ros::Time time_start_log;
#endif

class ImuCompiler {
	public:
		ImuCompiler(ros::NodeHandle nh, std::string imu_raw_topic, std::string imu_gravity_topic, 
			std::string imu_nogravity_topic, std::string frame_id
		) 
		: nh_(nh), imu_raw_topic_(imu_raw_topic), imu_gravity_topic_(imu_gravity_topic), imu_nogravity_topic_(imu_nogravity_topic)
		{
			imu_msg_.header.frame_id = frame_id;

			// Subscribes to raw imu data, publishes Imu message.
		    imu_raw_sub_ = nh_.subscribe(imu_raw_topic_, 1, &ImuCompiler::imuRawCallback, this);
		    imu_gravity_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_gravity_topic_, 1);
		    imu_nogravity_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_nogravity_topic_, 1);
		}

		void imuRawCallback(const std_msgs::Float32MultiArray::ConstPtr & imu_raw) {
			// Packet format is orientation [x][y][z][w], angular velocity [x][y][z], 
			//     raw acceleration [x][y][z], linear acceleration [x][y][z], timestamp [sec][nsec]

			// Copy over orientation and angular velocity data
			imu_msg_.orientation.x = imu_raw -> data[0];
			imu_msg_.orientation.y = imu_raw -> data[1];
			imu_msg_.orientation.z = imu_raw -> data[2];
			imu_msg_.orientation.w = imu_raw -> data[3];

			imu_msg_.angular_velocity.x = imu_raw -> data[6];
			imu_msg_.angular_velocity.y = imu_raw -> data[7];
			imu_msg_.angular_velocity.z = imu_raw -> data[8];

			// Extract timestamp. Needs to be cast from float back to uint32_t.
			ros::Time timestamp;
			// timestamp.sec = *const_cast<const uint32_t*>(reinterpret_cast<const uint32_t*>(&(imu_raw -> data[13])));
			// timestamp.nsec = *const_cast<const uint32_t*>(reinterpret_cast<const uint32_t*>(&(imu_raw -> data[14])));
			timestamp.sec = *const_cast<const uint32_t*>(reinterpret_cast<const uint32_t*>(&(imu_raw -> data[4])));
			timestamp.nsec = *const_cast<const uint32_t*>(reinterpret_cast<const uint32_t*>(&(imu_raw -> data[5])));
			imu_msg_.header.stamp = timestamp;
			
			// Update variance-covariance matrices
			// double roll, pitch, yaw;
			// toEulerAngle(
			// 	imu_msg_.orientation.x, imu_msg_.orientation.y, imu_msg_.orientation.z, imu_msg_.orientation.w,
			// 	roll, pitch, yaw
			// );
			// orientation_covariance.push(
			// 	roll, pitch, yaw
			// );
			// angular_velocity_covariance.push(
			// 	imu_msg_.angular_velocity.x, 
			// 	imu_msg_.angular_velocity.y, 
			// 	imu_msg_.angular_velocity.z
			// );
			// raw_acceleration_covariance.push(
			// 	imu_raw -> data[7],
			// 	imu_raw -> data[8],
			// 	imu_raw -> data[9]
			// );
			// linear_acceleration_covariance.push(
			// 	imu_raw -> data[10],
			// 	imu_raw -> data[11],
			// 	imu_raw -> data[12]
			// );

			// Get updated matrices
			//std::vector<double> o_cov = orientation_covariance.getCovMatrix();
			// std::vector<double> av_cov = angular_velocity_covariance.getCovMatrix();
			// std::vector<double> acc_cov = raw_acceleration_covariance.getCovMatrix();
			// std::vector<double> la_cov = linear_acceleration_covariance.getCovMatrix();

			// Copy over updated matrices
			for(int i=0; i<9; ++i) {
				imu_msg_.orientation_covariance[i] = 1e-6; //o_cov[i];
				imu_msg_.angular_velocity_covariance[i] = 1e-6; //av_cov[i];
			}

			// Now, copy over raw acceleration data and publish imu message with gravity vector included
			imu_msg_.linear_acceleration.x = imu_raw -> data[9];
			imu_msg_.linear_acceleration.y = imu_raw -> data[10];
			imu_msg_.linear_acceleration.z = imu_raw -> data[11];

			for(int i=0; i<9; ++i)
				imu_msg_.linear_acceleration_covariance[i] = 1e-6; //acc_cov[i];

			// Publish Imu message.
			imu_gravity_pub_.publish(imu_msg_);

			// Now, copy over acceleration data with gravity removed and publish imu message
			// imu_msg_.linear_acceleration.x = imu_raw -> data[10];
			// imu_msg_.linear_acceleration.y = imu_raw -> data[11];
			// imu_msg_.linear_acceleration.z = imu_raw -> data[12];

			// for(int i=0; i<9; ++i)
			// 	imu_msg_.linear_acceleration_covariance[i] = la_cov[i];

			// Publish Imu message.
			//imu_nogravity_pub_.publish(imu_msg_);
		}

		// Used to calculate orientation covariance matrix
		void toEulerAngle(double x, double y, double z, double w, double& roll, double& pitch, double& yaw) {
			// roll (x-axis rotation)
			double sinr = +2.0 * (w * x + y * z);
			double cosr = +1.0 - 2.0 * (x * x + y * y);
			roll = atan2(sinr, cosr);

			// pitch (y-axis rotation)
			double sinp = +2.0 * (w * y - z * x);
		    if (fabs(sinp) >= 1)
		        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
		    else
		    	pitch = asin(sinp);

			// yaw (z-axis rotation)
			double siny = +2.0 * (w * z + x * y);
			double cosy = +1.0 - 2.0 * (y * y + z * z);  
			yaw = atan2(siny, cosy);
		}

	private:
		ros::NodeHandle nh_;
		std::string imu_raw_topic_;
		std::string imu_gravity_topic_;
		std::string imu_nogravity_topic_;
		sensor_msgs::Imu imu_msg_;
		ros::Subscriber imu_raw_sub_;
		ros::Publisher imu_gravity_pub_;
		ros::Publisher imu_nogravity_pub_;
		covarianceMatrix orientation_covariance;
		covarianceMatrix angular_velocity_covariance;
		covarianceMatrix raw_acceleration_covariance;
		covarianceMatrix linear_acceleration_covariance;
};


int main(int argc, char** argv) {
	// Setup the ROS node
	ros::init(argc, argv, "xv11_raw2imu_node");
	ros::NodeHandle nh;
	ros::NodeHandle pr_nh("~"); // Private node handle to get params

	// std::string topic_postfix;
	// // Retrieve specified postfix, if any.
	// if( !pr_nh.getParam("topic_postfix", topic_postfix) ){
	// 	topic_postfix = "";
	// }

	// Start up compiler.
	ImuCompiler xv11_imu_compiler(
		nh,
		"imu_data_raw",
		"imu_gravity",
		"imu_nogravity",
		"imu_link"
	);
	ros::spin();

	return 0;
}