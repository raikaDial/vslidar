#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

class ICPCloudAlignment {
	public:
		ICPCloudAlignment(ros::NodeHandle nh) : 
			nh_(nh), cloud_1(new pcl::PointCloud<pcl::PointXYZ>), cloud_2(new pcl::PointCloud<pcl::PointXYZ>),
			listener_(tf_buffer_)
		{
			cloud_sub_1 = nh_.subscribe("cloud_1", 1, &ICPCloudAlignment::cloudCallback, this);
			cloud_sub_2 = nh_.subscribe("cloud_2", 1, &ICPCloudAlignment::cloudCallback, this);
			cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("clouds_merged", 1);

			timer = nh.createTimer(ros::Duration(30.0), &ICPCloudAlignment::timerCallback, this);

		}

		// Variables
		ros::NodeHandle nh_;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2;

		tf2_ros::Buffer tf_buffer_;
		tf2_ros::TransformListener listener_;

		ros::Subscriber cloud_sub_1;
		ros::Subscriber cloud_sub_2;
		ros::Publisher cloud_pub;

		ros::Timer timer;

		// Functions
		void print4x4Matrix (const Eigen::Matrix4d & matrix) {
		  printf ("Rotation matrix :\n");
		  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
		  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
		  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
		  printf ("Translation vector :\n");
		  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
		}

		void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in) {
			// Store point clouds
			pcl::PointCloud<pcl::PointXYZ> cloud_buffer;
			geometry_msgs::TransformStamped transform_stamped;

			if((cloud_in -> header.frame_id) == "base_laser_1") {
				// Look up static transform as initial guess for ICP
				try {
					transform_stamped = tf_buffer_.lookupTransform(
						"base_laser", "base_laser_1", ros::Time(0)
					);

				}
				catch (tf2::TransformException &ex) {
		   			ROS_WARN("%s",ex.what());
		   			return;
		   		}

		   		sensor_msgs::PointCloud2 cloud_transformed;
		   		tf2::doTransform(*cloud_in, cloud_transformed, transform_stamped);

				pcl::fromROSMsg(cloud_transformed, cloud_buffer);
				*cloud_1 += cloud_buffer;
			}
			else if((cloud_in -> header.frame_id) == "base_laser_2") {
				try {
					transform_stamped = tf_buffer_.lookupTransform(
						"base_laser", "base_laser_2", ros::Time(0)
					);

				}
				catch (tf2::TransformException &ex) {
		   			ROS_WARN("%s",ex.what());
		   			return;
		   		}

		   		sensor_msgs::PointCloud2 cloud_transformed;
		   		tf2::doTransform(*cloud_in, cloud_transformed, transform_stamped);

				pcl::fromROSMsg(cloud_transformed, cloud_buffer);
				*cloud_2 += cloud_buffer;
			}
		}

		void timerCallback(const ros::TimerEvent& event) {
			// If we have collected both clouds, perform ICP and publish transform
			if(!(cloud_1 -> empty()) && !(cloud_2 -> empty())) {
				// Perform ICP to find best transform between the two clouds
				pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
				icp.setMaximumIterations(100);
				icp.setInputSource(cloud_1);
				icp.setInputTarget(cloud_2);
				icp.align(*cloud_1);

				if(icp.hasConverged()) {
					// Print the transform
					ROS_INFO("ICP has converged, score is %f", icp.getFitnessScore());
				  	Eigen::Matrix4d transformation_matrix = icp.getFinalTransformation ().cast<double>();
				  	print4x4Matrix (transformation_matrix);

				  	// Do the transform and publish the merged cloud.
					pcl::transformPointCloud(*cloud_1, *cloud_1, transformation_matrix);
				  	pcl::PointCloud<pcl::PointXYZ> cloud_merged = *cloud_1 + *cloud_2;
				  	sensor_msgs::PointCloud2 cloud_merged_msg;
				  	pcl::toROSMsg(cloud_merged, cloud_merged_msg);
				  	cloud_merged_msg.header.frame_id = "base_laser_2";

				  	cloud_pub.publish(cloud_merged_msg);
				}

				cloud_1 -> clear();
				cloud_2 -> clear();
			}
		}
};

int main(int argc, char** argv) {
	// Setup the ROS node
	ros::init(argc, argv, "icp_node");
	ros::NodeHandle nh;

	ICPCloudAlignment icp_cloud_alignment(nh);

	ros::spin();

	return 0;
}