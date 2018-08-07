// cloud2obstaclemap_node.cpp
// This node, given point clouds, creates and publishes a map containing the obstacles.

// Ryker Dial
// Date Created: February 8, 2018
// Last Modified: February 9, 2018

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
// #include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// #include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <string>

// This cloud accumulates point clouds and publishes combined clouds at a set interval.
class Cloud2ObstacleMap {
	public:
		ros::NodeHandle nh_;
		ros::Subscriber cloud_sub_1_;
		ros::Subscriber cloud_sub_2_;
		ros::Subscriber cloud_accumulated_sub_1_;
		ros::Subscriber	cloud_accumulated_sub_2_;
		ros::Publisher cloud_obstacles_marking_pub_;
		ros::Publisher cloud_obstacles_clearing_pub_;

		tf2_ros::TransformListener listener_;
		tf2_ros::Buffer tf_buffer_; // Buffer stores received transforms

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_accumulated_;
		pcl::ModelCoefficients::Ptr coefficients;

		double publish_frequency_;
		ros::Timer publish_timer_;

		// Passthrough filter to create obstacle cloud
		pcl::PassThrough<pcl::PointXYZ> pass_floor_; // Removes points between min_obstacle_depth_ and min_obstacle_height
		pcl::PassThrough<pcl::PointXYZ> pass_; // Removes points that aren't within max_obstacle_depth_ and max_obstacle_height_;
		pcl::ProjectInliers<pcl::PointXYZ> proj_;
		double min_obstacle_height_; // Minimum height at which a point is considered an obstacle
		double max_obstacle_height_;
		double min_obstacle_depth_; // Minimum depth at which a point is considered an obstacle;
		double max_obstacle_depth_;

		Cloud2ObstacleMap(ros::NodeHandle n) 
			: nh_(n), listener_(tf_buffer_), cloud_accumulated_(new pcl::PointCloud<pcl::PointXYZ>), coefficients(new pcl::ModelCoefficients ())
		{
			// Get Mapping Parameters
			ros::NodeHandle pr_nh("~");
			pr_nh.param<double>("publish_frequency", publish_frequency_, 5.0);
			pr_nh.param<double>("min_obstacle_height", min_obstacle_height_, 0.05);
			pr_nh.param<double>("max_obstacle_height", max_obstacle_height_, 1.0);
			pr_nh.param<double>("min_obstace_depth", min_obstacle_depth_, -0.05);
			pr_nh.param<double>("max_obsacle_depth", max_obstacle_depth_, -1.0);

			// Initialize passthrough filter
			pass_floor_.setFilterFieldName("z");
			pass_floor_.setFilterLimits(min_obstacle_depth_, min_obstacle_height_);
			pass_floor_.setNegative(true);

			pass_.setFilterFieldName("z");
			pass_.setFilterLimits(max_obstacle_depth_, max_obstacle_height_);

			// Initialize projector planar coefficients with X=Y=0, Z=1
			// This will project the cloud points into the XY plane
			coefficients->values.resize (4);
			coefficients->values[0] = coefficients->values[1] = 0;
			coefficients->values[2] = 1.0;
			coefficients->values[3] = 0;
			proj_.setModelType(pcl::SACMODEL_PLANE);
			proj_.setModelCoefficients(coefficients);

			cloud_sub_1_ = nh_.subscribe("cloud_1", 1000, &Cloud2ObstacleMap::cloudCallback, this);
			cloud_sub_2_ = nh_.subscribe("cloud_2", 1000, &Cloud2ObstacleMap::cloudCallback, this);
			cloud_accumulated_sub_1_ = nh_.subscribe("cloud_accumulated_1", 1000, &Cloud2ObstacleMap::cloudAccumulatedCallback, this);
			cloud_accumulated_sub_2_ = nh_.subscribe("cloud_accumulated_2", 1000, &Cloud2ObstacleMap::cloudAccumulatedCallback, this);

			cloud_obstacles_marking_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_obstacles_marking", 10);
			cloud_obstacles_clearing_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_obstacles_clearing", 10);

			// Start Publish Timer
			publish_timer_ = nh_.createTimer(ros::Duration(1.0/publish_frequency_), &Cloud2ObstacleMap::publishTimerCallback, this);

		}

		void filterObstacles(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in) {
			// Create buffer clouds
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_1 (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZ>);

			// Remove points belonging to the ground plane
			pass_floor_.setInputCloud(cloud_in);
			pass_floor_.filter(*cloud_filtered_1);

			// Remove points on ceiling or hanging obstacles robot can pass under.
			pass_.setInputCloud(cloud_filtered_1);
			//pass_.filter(*cloud_filtered_2);
			pass_.filter(*cloud_in);

			// Project remaining obstacles to the ground plane
			// proj_.setInputCloud(cloud_filtered_2);
			// proj_.filter(*cloud_in);				
		}

		void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in) {
			// Transform the cloud's coordinate frame from base_footprint to odom.
            geometry_msgs::TransformStamped transform_stamped;
			try {
				transform_stamped = tf_buffer_.lookupTransform(
					"odom", cloud_in -> header.frame_id, ros::Time(0)
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

		void cloudAccumulatedCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in) {
			// Transform cloud to pcl point cloud
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_buffer (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(*cloud_in, *cloud_buffer);

			if(!(cloud_buffer -> empty())) {
				// Filter the cloud
				filterObstacles(cloud_buffer);

				// If the cloud isn't empty, publish it
				if(!(cloud_buffer -> empty())) {
			   		sensor_msgs::PointCloud2 cloud_obstacles_clearing_msg;
					pcl::toROSMsg(*cloud_buffer, cloud_obstacles_clearing_msg);

					// geometry_msgs::TransformStamped transform_stamped;
					// try {
					// 	transform_stamped = tf_buffer_.lookupTransform(
					// 		"base_footprint", "odom", ros::Time(0)
					// 	);
					// }
					// catch (tf2::TransformException &ex) {
			  //  			ROS_WARN("%s",ex.what());
			  //  			return;
			  //  		}
			  //  		tf2::doTransform(cloud_obstacles_clearing_msg, cloud_obstacles_clearing_msg, transform_stamped);

					// cloud_obstacles_clearing_msg.header.frame_id = "base_footprint";
					cloud_obstacles_clearing_msg.header.stamp = ros::Time::now();
					cloud_obstacles_clearing_pub_.publish(cloud_obstacles_clearing_msg);
				}
			}
		}

		void publishTimerCallback(const ros::TimerEvent& event) {
			if(!(cloud_accumulated_ -> empty())) {
				filterObstacles(cloud_accumulated_);

				if(!(cloud_accumulated_ -> empty())) {
			   		sensor_msgs::PointCloud2 cloud_obstacles_marking_msg;
					pcl::toROSMsg(*cloud_accumulated_, cloud_obstacles_marking_msg);

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
			   		tf2::doTransform(cloud_obstacles_marking_msg, cloud_obstacles_marking_msg, transform_stamped);

					cloud_obstacles_marking_msg.header.frame_id = "base_footprint";
					cloud_obstacles_marking_msg.header.stamp = ros::Time::now();
					cloud_obstacles_marking_pub_.publish(cloud_obstacles_marking_msg);
				}
				cloud_accumulated_ -> clear();
			}
		}
};

int main(int argc, char** argv) {

	// Setup the ROS node.
	ros::init(argc, argv, "cloud2obstaclemap_node");
	ros::NodeHandle nh;

	// Start the accumulator
	Cloud2ObstacleMap cloud_2_obstacle_map(nh);
	ros::spin();

	return 0;
}