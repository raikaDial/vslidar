// cloud_accumulator_node.cpp
// Accumulates point clouds published to the "cloud_obstacles" topic for a certain interval, 
//     then publishes the accumulated cloud to "points2" for use by Google Cartographer.

// Ryker Dial
// Date Created: September 21, 2017
// Last Modified: March 8, 2018

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

// Collects a complete panning cycle's worth of point clouds and sends them to cartographer for localization
class CloudAccumulator {
	public:
		ros::NodeHandle nh_;
		//ros::Subscriber cloud_sub_1_, cloud_sub_2_, cloud_obstacles_sub_;
		ros::Subscriber cloud_sub_;
		ros::Subscriber pan_finished_signal_sub_;
		ros::Publisher cloud_accumulated_pub_;

		std::string cloud_frame;

		tf2_ros::TransformListener listener_;
		tf2_ros::Buffer tf_buffer_; // Buffer stores received transforms

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_accumulated_;

		CloudAccumulator(ros::NodeHandle nh) 
			: nh_(nh), listener_(tf_buffer_), cloud_accumulated_(new pcl::PointCloud<pcl::PointXYZ>)
		{
			ros::NodeHandle pr_nh("~");
		  	// Retrieve specified postfix, if any, to uniquely identify topics and transform frames
			std::string topic_postfix;
			if( !pr_nh.getParam("topic_postfix", topic_postfix) ) {
				topic_postfix = "";
			}

			cloud_accumulated_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_accumulated"+topic_postfix, 1);
			cloud_sub_ = nh_.subscribe("cloud"+topic_postfix, 1000, &CloudAccumulator::cloudCallback, this);
			pan_finished_signal_sub_ = nh_.subscribe("ax12_done_panning"+topic_postfix, 1, &CloudAccumulator::panFinishedCallback, this);
		}

		void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in) {
			// Transform the cloud's coordinate frame from base_footprint to odom.
			cloud_frame = cloud_in -> header.frame_id;
            geometry_msgs::TransformStamped transform_stamped;
			try {
				transform_stamped = tf_buffer_.lookupTransform(
					"odom", cloud_frame, ros::Time(0)
				);
			}
			catch (tf2::TransformException &ex) {
	   			ROS_WARN("%s",ex.what());
	   			return;
	   		}
	   		sensor_msgs::PointCloud2 cloud_transformed;
	   		tf2::doTransform(*cloud_in, cloud_transformed, transform_stamped);

			pcl::PointCloud<pcl::PointXYZ> cloud_buffer;
			pcl::fromROSMsg(cloud_transformed, cloud_buffer);
		
			*cloud_accumulated_ += cloud_buffer;
		}

		void panFinishedCallback(const std_msgs::Empty::ConstPtr & msg) {
			if(!(cloud_accumulated_ -> empty())) {
				// Transform merged cloud back to base_footprint frame so SLAM has a reference point
				geometry_msgs::TransformStamped transform_stamped;
				try {
					transform_stamped = tf_buffer_.lookupTransform(
						"base_footprint", "odom", ros::Time(0)
					);
				}
				catch (tf2::TransformException &ex) {
		   			ROS_WARN("%s",ex.what());
		   			return;
		   		}
		   		sensor_msgs::PointCloud2 cloud_accumulated_msg;
				pcl::toROSMsg(*cloud_accumulated_, cloud_accumulated_msg);
				tf2::doTransform(cloud_accumulated_msg, cloud_accumulated_msg, transform_stamped);

				// Publish cloud for cartographer
				cloud_accumulated_msg.header.frame_id = "base_footprint";
				cloud_accumulated_msg.header.stamp = ros::Time::now();
				cloud_accumulated_pub_.publish(cloud_accumulated_msg);			

				cloud_accumulated_ -> clear();
			}
		}
};

int main(int argc, char** argv) {

	// Setup the ROS node.
	ros::init(argc, argv, "cloud_accumulator_node");
	ros::NodeHandle nh;

	// Start the accumulator
	CloudAccumulator cloud_accumulator(nh);
	ros::spin();

	return 0;
}