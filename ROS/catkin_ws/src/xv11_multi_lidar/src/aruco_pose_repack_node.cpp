// aruco_pose_repack_node.cpp
// Subscribes to aruco pose message and repackages it as a geometry_msgs/PoseWithCovarianceStamped

// Ryker Dial
// Date Created: February 26, 2018
// Last Modified: February 26, 2018

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <aruco_mapping/ArucoMarker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>

class ArucoPoseRepacker {
	public:
		ArucoPoseRepacker(ros::NodeHandle nh) : nh_(nh), listener_(tf_buffer_), have_camera_offset_(false) {
			pose_sub_ = nh_.subscribe("aruco_poses", 10, &ArucoPoseRepacker::arucoPoseCallback, this);
			pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("aruco_pose_cov",1);
		}

		void arucoPoseCallback(const aruco_mapping::ArucoMarker::ConstPtr & aruco_markers) {
			if(!have_camera_offset_) { // Make sure we have fetched the camera offset from the center of rotation.
				try {
					// Get offset of usb camera from the center of rotation
	                geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
	                	"base_link", "usb_cam", ros::Time(0)
	                );
	                double x_offset = transform_stamped.transform.translation.x;
	                double y_offset = transform_stamped.transform.translation.y;
	                camera_offset_ = sqrt(x_offset*x_offset + y_offset*y_offset);
	                have_camera_offset_ = true;
	            }
	            catch (tf2::TransformException& ex) {
	                ROS_WARN("%s",ex.what());
	                return;
	            }				
			}
			if((aruco_markers -> marker_visible) && have_camera_offset_) {
				// TODO: Check the ID

				// TODO: Check distance away from marker


				// Get angle to marker
				double roll, pitch, yaw;
				toEulerAngle(
					aruco_markers -> global_camera_pose.orientation.x, aruco_markers -> global_camera_pose.orientation.y,
					aruco_markers -> global_camera_pose.orientation.z, aruco_markers -> global_camera_pose.orientation.w,
					roll, pitch, yaw
				);

				// Get corrected marker coordinates
				double x_pos = -(aruco_markers -> global_camera_pose.position.z + camera_offset_*cos(pitch));
				double y_pos = (aruco_markers -> global_camera_pose.position.y - camera_offset_*sin(pitch));

				double distance = sqrt(x_pos*x_pos + y_pos*y_pos);
				if(distance <= 1.0) {

					// Publish marker pose as the absolute world pose of the robot
					geometry_msgs::PoseWithCovarianceStamped pose_msg;

					// Populate Header
					pose_msg.header.stamp = aruco_markers -> header.stamp;
					pose_msg.header.seq = aruco_markers -> header.seq;
		            pose_msg.header.frame_id = "odom";

		            // Store 2D Pose Information
		            pose_msg.pose.pose.position.x = x_pos;
		            pose_msg.pose.pose.position.y = y_pos;
		            tf2::Quaternion q;
		            q.setRPY(0, 0, pitch);
		            pose_msg.pose.pose.orientation.x = q.x();
		            pose_msg.pose.pose.orientation.y = q.y();
		            pose_msg.pose.pose.orientation.z = q.z();
		            pose_msg.pose.pose.orientation.w = q.w();

		            //Populate covariance matrix with fixed low covariance
					for(int i=0; i<36; ++i) {
						pose_msg.pose.covariance [i] = 1e-6;
					}	            

					// Transmit pose
					pose_pub_.publish(pose_msg);
				}
			}
		}

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
		ros::Subscriber pose_sub_;
		ros::Publisher pose_pub_;

		tf2_ros::TransformListener listener_;
        tf2_ros::Buffer tf_buffer_; // Buffer stores received transforms

        bool have_camera_offset_;
        double camera_offset_;
};

int main(int argc, char** argv) {
	// Setup the ROS node.
	ros::init(argc, argv, "aruco_pose_repack_node");
	ros::NodeHandle nh;

	// Start up the repacker
	ArucoPoseRepacker aruco_pose_repacker(nh);
	ros::spin();

	return 0;
}