// xv11_raw2scan_wtf_node.cpp
// Takes the packets of raw distance and intensity values from the LIDARs and converts them into LaserScan packets.
// Also broadcasts the pan angle transform.

// Author: Ryker Dial
// Created: July 25, 2017
// Last Modified: February 3, 2018

#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>

#define _USE_MATH_DEFINES
#include <cmath>

class Xv11LaserScanCompiler {
	public:
		Xv11LaserScanCompiler(ros::NodeHandle nh, std::string scan_raw_topic, std::string scan_topic, std::string scan_frame, std::string topic_postfix) 
		: nh_(nh), scan_raw_topic_(scan_raw_topic + topic_postfix), scan_topic_(scan_topic + topic_postfix), scan_frame_(scan_frame + topic_postfix),
		  topic_postfix_(topic_postfix), laser_offset_rotation_(0.037), laser_offset_base_(0.122)
		{
			// Get params
           	ros::NodeHandle pr_nh("~");
            pr_nh.param<bool>("rectify_distance", rectify_distance_, true);

			// Setup the LaserScan message
			scan_msg_.header.frame_id = scan_frame_; // Set LaserScan frame id.
			scan_msg_.angle_increment = M_PI/180.0; // One degree increments

		    // Acceptable range of measurements is 2.0cm to 6.0m according to whitepaper.
		    scan_msg_.range_min = 0.02;
		    scan_msg_.range_max = 6.0;

		    // Subscribes to raw scan data, publishes LaserScan.
		    scan_raw_sub_ = nh_.subscribe(scan_raw_topic_, 1000, &Xv11LaserScanCompiler::scanRawCallback, this);
		    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_, 1000);
		    angle_pub_ = nh_.advertise<std_msgs::Float32>("ax12_angle" + topic_postfix_, 1000);
		    motor_rpm_pub_ = nh_.advertise<std_msgs::Float32>("motor_rpm" + topic_postfix_, 1000);
		}

		void broadcastPanAngleTransform(const float & angle) {
			static tf2_ros::TransformBroadcaster br;

			// Create the transform and set the header information
			geometry_msgs::TransformStamped transform_stamped;
			transform_stamped.header.frame_id = "base_laser" + topic_postfix_;
			transform_stamped.child_frame_id = "laser_frame" + topic_postfix_;

			// Modify the origin of the LaserScan data to match the laser's offset from the center of 
			//     rotation about the servo and its distance from the servo base.
			transform_stamped.transform.translation.x = laser_offset_base_;
			transform_stamped.transform.translation.y = laser_offset_rotation_*cos( (angle-60.0)*M_PI/180.0 );
			transform_stamped.transform.translation.z = laser_offset_rotation_*sin( (angle-60.0)*M_PI/180.0 );

			// Rotate the LaserScan data based upon the panning angle of the servo to create rotated 
			//     planes of points which can be combined into a point cloud.
			// To align z-axis of the laser's frame with the z-axis of the laser base's frame, need to rotate about
			//     the x-axis by the angle complementary to the servo angle.
			tf2::Quaternion q;
			q.setRPY(-(90.0 - (angle - 60.0))*M_PI/180.0, 0.0, 0.0);
			transform_stamped.transform.rotation.x = q.x();
			transform_stamped.transform.rotation.y = q.y();
			transform_stamped.transform.rotation.z = q.z();
			transform_stamped.transform.rotation.w = q.w();

			
			// Timestamp transform. Timestamp is starting time of scan + time it took to collect the data
			transform_stamped.header.stamp = scan_msg_.header.stamp;
			if(!scan_msg_.ranges.empty()) {
				transform_stamped.header.stamp += ros::Duration().fromSec((scan_msg_.ranges.size()-1)*scan_msg_.time_increment);
			}

			// Transmit the transform
			br.sendTransform(transform_stamped);
		}		

		void scanRawCallback(const std_msgs::UInt16MultiArray::ConstPtr & scan_raw) {
			// Packet format is [num_vals_scan distance][num_vals_scan intensity][starting angle][timestamp secs][timestamp nsecs][servo angle][avg motor rpm]

			int num_vals_scan = (scan_raw -> data.size() - 9) / 2; // Get number of scan data points.

			// Extract starting angle and calculate ending angle
			// For some arcane, unknowable reason, we get twisting in the point cloud if we don't subtract 2 degrees from
			//     the starting angle. This holds true regardless of what RPM the LIDAR is spinning at.
			scan_msg_.angle_min = ((scan_raw -> data.end()[-9])-2)*M_PI/180.0;
			//scan_msg_.angle_min = ((scan_raw -> data.end()[-9]))*M_PI/180.0;
			scan_msg_.angle_max = fmod(scan_msg_.angle_min + scan_msg_.angle_increment*(num_vals_scan-1), 2*M_PI);

			// Extract Timestamp
			scan_msg_.header.stamp.sec = (scan_raw -> data.end()[-8]) << 16 | (scan_raw -> data.end()[-7]);
			scan_msg_.header.stamp.nsec = (scan_raw -> data.end()[-6]) << 16 | (scan_raw -> data.end()[-5]);

			// Make sure timestamp is valid
			try {
				if(fabs((ros::Time::now() - scan_msg_.header.stamp).toSec()) > 1.0) {
					scan_msg_.header.stamp = ros::Time::now();
				}
			}
			catch(std::runtime_error &e) {
				// Timestamp was somehow corrupted on the MCU before being stored in the message
				scan_msg_.header.stamp = ros::Time::now();
			}

			//ROS_INFO("Raw Timestamp: %u secs, %u nsecs", scan_msg_.header.stamp.sec, scan_msg_.header.stamp.nsec);

			// Extract Servo Angle
			uint32_t angle_bytes = (scan_raw -> data.end()[-4]) << 16 | (scan_raw -> data.end()[-3]);
			float angle = *const_cast<const float*>(reinterpret_cast<const float*>(&angle_bytes));

			if(angle < 0) {
				angle = _last_angle_received;
			}
			else {
				_last_angle_received = angle;
			}

			std_msgs::Float32 angle_msg;
			angle_msg.data = angle;
			//ROS_INFO("Servo Angle %s: %f", topic_postfix_.c_str(), angle);

			// Extract motor rpm and estimate time increment between measurements
			uint32_t motor_rpm_avg_bytes = (scan_raw -> data.end()[-2]) << 16 | (scan_raw -> data.end()[-1]);
			float motor_rpm_avg = *const_cast<const float*>(reinterpret_cast<const float*>(&motor_rpm_avg_bytes));
			if(motor_rpm_avg != 0) { // Check just in case something weird happened
				scan_msg_.time_increment = 1.0/(motor_rpm_avg/60*360); // (revs/second * 360 meas/rev)^(-1)
			}
			
			std_msgs::Float32 motor_rpm_msg;
			motor_rpm_msg.data = motor_rpm_avg;

			//ROS_INFO("Motor RPM %s: %f", topic_postfix_.c_str(), motor_rpm_avg);

			// // Make sure arrays are properly sized
		    scan_msg_.ranges.resize(num_vals_scan);
		    scan_msg_.intensities.resize(num_vals_scan);

		    // Populate LaserScan message with values.
		    for(int i=0; i<num_vals_scan; ++i) {
		    	float distance = (scan_raw -> data[i])/1000.0; // convert to meters

		    	// If reported measurement is out of range, set distance and intensity to zero.
		    	if(distance < scan_msg_.range_min || distance > scan_msg_.range_max) {
		    		scan_msg_.ranges[i] = 0;
		    		scan_msg_.intensities[i] = 0;
		    	}
		    	else { // Measurements are good, store them.
		    		if(rectify_distance_) {
		    			scan_msg_.ranges[i] = -0.0295*distance*distance + 1.0361*distance - 0.0057;
		    		}
		    		else {
		    			scan_msg_.ranges[i] = distance;
		    		}
		    		scan_msg_.intensities[i] = scan_raw -> data[num_vals_scan + i];
		    	}
		    }

		    // Publish LaserScan message and servo angle transform
		    broadcastPanAngleTransform(angle);
			scan_pub_.publish(scan_msg_);
			angle_pub_.publish(angle_msg);
			motor_rpm_pub_.publish(motor_rpm_msg);

		}

	private:
		ros::NodeHandle nh_;
		std::string scan_raw_topic_;
		std::string scan_topic_;
		std::string scan_frame_;
		std::string topic_postfix_;
		sensor_msgs::LaserScan scan_msg_;
		ros::Subscriber scan_raw_sub_;
		ros::Publisher scan_pub_;

		ros::Publisher motor_rpm_pub_;
		ros::Publisher angle_pub_; // for debug
		double _last_angle_received; // Store the last good angle received in case of servo read error

		double laser_offset_rotation_; // Laser's offset from center of rotation in meters.
		double laser_offset_base_; // Laser's offset to the base of the servo in meters.

		bool rectify_distance_;
};


int main(int argc, char** argv) {
	// Setup the ROS node
	ros::init(argc, argv, "xv11_raw2scan_wtf_node");
	ros::NodeHandle nh;
	ros::NodeHandle pr_nh("~"); // Private node handle to get params

	std::string topic_postfix;
	// Retrieve specified postfix, if any.
	if( !pr_nh.getParam("topic_postfix", topic_postfix) ){
		topic_postfix = "";
	}

	// Start up compiler.
	Xv11LaserScanCompiler xv11_scan_compiler(
		nh,
		"xv11_scan_raw",
		"xv11_scan",
		"laser_frame",
		topic_postfix
	);
	ros::spin();

	return 0;
}