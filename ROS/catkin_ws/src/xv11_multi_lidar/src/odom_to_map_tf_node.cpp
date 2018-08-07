// odom_to_map_tf_node.cpp
// Uses IMU orientation data to broadcast the rotational transform between the 
//     odom and map frames. This allows maps to be created with 2D odometry more
//     accurately.

// Ryker Dial
// Date Created: February 16, 2018
// Last Modified: February 16, 2018

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>

class MapToOdomTfBroadcaster {
	public:
		MapToOdomTfBroadcaster(ros::NodeHandle nh) : nh_(nh), listener_(tf_buffer_) {
			imu_sub_ = nh_.subscribe("imu", 1, &MapToOdomTfBroadcaster::broadcastIMUTransform, this);
		}

		void broadcastIMUTransform(const sensor_msgs::Imu::ConstPtr & imu) {
			static tf2_ros::TransformBroadcaster br;

			// Create the transform and set the header information
			geometry_msgs::TransformStamped odom_to_map_stamped;
			odom_to_map_stamped.header.frame_id = "map";
			odom_to_map_stamped.child_frame_id = "odom";

			// Set translations to zero
			odom_to_map_stamped.transform.translation.x = 0.0;
			odom_to_map_stamped.transform.translation.y = 0.0;
			odom_to_map_stamped.transform.translation.z = 0.0;

			// Rotate map frame relative to odom (and by extension the robot) using the IMU
			tf2::Quaternion q1(
				imu -> orientation.x, 
				imu -> orientation.y,
				imu -> orientation.z,
				imu -> orientation.w
			);

			geometry_msgs::TransformStamped transform_stamped;
			try {
				transform_stamped = tf_buffer_.lookupTransform(
					"odom", "imu_link", ros::Time(0)
				);
			}
			catch (tf2::TransformException &ex) {
	   			ROS_WARN("%s",ex.what());
	   			return;
	   		}

	   		tf2::Quaternion q2(
	   			transform_stamped.transform.rotation.x,
	   			transform_stamped.transform.rotation.y,
	   			transform_stamped.transform.rotation.z,
	   			transform_stamped.transform.rotation.w
	   		);

	   		q1 *= q2;
	   		q1.normalize();

			// Convert quaternion to RPY, remove yaw, then convert back to quaternion
			tf2::Matrix3x3 m(q1);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			q1.setRPY(roll, pitch, 0);

	   		// Store transform
			odom_to_map_stamped.transform.rotation.x = q1.x();
			odom_to_map_stamped.transform.rotation.y = q1.y();
			odom_to_map_stamped.transform.rotation.z = q1.z();
			odom_to_map_stamped.transform.rotation.w = q1.w();

			// Timestamp and transmit transform
			odom_to_map_stamped.header.stamp = imu -> header.stamp;
			br.sendTransform(odom_to_map_stamped);
		}

	private:
		ros::NodeHandle nh_;
		ros::Subscriber imu_sub_;

		tf2_ros::TransformListener listener_;
		tf2_ros::Buffer tf_buffer_;
};



int main(int argc, char** argv) {
	// Setup the ROS node
	ros::init(argc, argv, "odom_to_map_tf_node");
	ros::NodeHandle nh;

	MapToOdomTfBroadcaster map_to_odom_tf_br(nh);

	ros::spin();
	return 0;
}