// scan2cloud_node.cpp
// Modified version of LaserScan to Point Cloud tutorial found here:
//     http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData

// Ryker Dial
// Last Modified: February 9, 2018

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/TransformStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <string>
#include <iostream>

// Class subscribes to a sensor_msgs::LaserScan topic and a tf2 topic, and uses the tf
//     to transform the LaserScan to a sensor_msgs::PointCloud2.
class LaserScanToPointCloud2 {
    public:
        ros::NodeHandle n_;
        laser_geometry::LaserProjection projector_;
        tf2_ros::TransformListener listener_;
        tf2_ros::Buffer tf_buffer_; // Buffer stores received transforms
        message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
        tf2_ros::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
        ros::Publisher cloud_pub_;

        std::string scan_topic_;
        std::string scan_tf_;
        std::string cloud_topic_;
        std::string topic_postfix_;

        pcl::PassThrough<pcl::PointXYZ> pass_z_; // Filter to remove points outside of z-range
        pcl::CropBox<pcl::PointXYZ> crop_bounding_box_; // Filter to remove points within robot's bounding box;

        // Dimensions of bounding box in meters for the robot for filtering the cloud
        double ROBOT_X_;
        double ROBOT_Y_;
        double ROBOT_Z_;

        double PASS_FILTER_MIN_Z_;
        double PASS_FILTER_MAX_Z_;

        LaserScanToPointCloud2(ros::NodeHandle n, std::string scan_topic, std::string scan_tf, std::string cloud_topic, std::string topic_postfix) : 
            n_(n), laser_sub_(n_, scan_topic, 10), listener_(tf_buffer_), laser_notifier_(laser_sub_, tf_buffer_, scan_tf, 10, 0),
            scan_topic_(scan_topic), scan_tf_(scan_tf), cloud_topic_(cloud_topic), topic_postfix_(topic_postfix) {

            // Get cloud filtering parameters
            ros::NodeHandle pr_nh("~");
            pr_nh.param<double>("robot_x", ROBOT_X_, 0.33);
            pr_nh.param<double>("robot_y", ROBOT_Y_, 0.46);
            pr_nh.param<double>("robot_z", ROBOT_Z_, 0.80);
            pr_nh.param<double>("pass_filter_min_z", PASS_FILTER_MIN_Z_, -10);
            pr_nh.param<double>("pass_filter_max_z", PASS_FILTER_MAX_Z_, 10);

            pass_z_.setFilterFieldName("z");
            pass_z_.setFilterLimits(PASS_FILTER_MIN_Z_, PASS_FILTER_MAX_Z_); // Only keep the points whose z-value is between the limits

            crop_bounding_box_.setMin(Eigen::Vector4f(-ROBOT_X_/2, -ROBOT_Y_/2, 0, 1.0));
            crop_bounding_box_.setMax(Eigen::Vector4f(ROBOT_X_/2, ROBOT_Y_/2, ROBOT_Z_, 1.0));
            crop_bounding_box_.setNegative(true);

            laser_notifier_.registerCallback(
                boost::bind(&LaserScanToPointCloud2::scanCallback, this, _1)
            );
            laser_notifier_.setTolerance(ros::Duration(0.01));
            cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>(cloud_topic_, 1);
        }

        void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in) {
            // Transform the laser scan into a point cloud.
            sensor_msgs::PointCloud2 cloud;

            try {
                tf_buffer_.lookupTransform( // Wait to make sure the transform is available.
                    "base_footprint", "laser_frame" + topic_postfix_, 
                    scan_in -> header.stamp + ros::Duration().fromSec((scan_in -> ranges.size()-1)*scan_in -> time_increment)
                );
                projector_.transformLaserScanToPointCloud(
                    scan_tf_, *scan_in, cloud, tf_buffer_
                );
            }
            catch (tf2::TransformException& ex) {
                ROS_WARN("%s",ex.what());
                return;
            }

            // Filter the point cloud
            // These are boost pointers, so no need to call delete.
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(cloud, *cloud_pcl);

            // Remove points that are too high or too low to be useful
            pass_z_.setInputCloud(cloud_pcl);
            pass_z_.filter(*cloud_filtered); // Remove all points that are higher than the z-limit

            // Remove points within the robot's bounding box.
            crop_bounding_box_.setInputCloud(cloud_filtered);
            crop_bounding_box_.filter(*cloud_pcl);

            // Transform back to ROS message
            pcl::toROSMsg(*cloud_pcl, cloud);

            // Transform the cloud's coordinate frame back to laser_frame_*
            geometry_msgs::TransformStamped transform_stamped;
            try {
                transform_stamped = tf_buffer_.lookupTransform(
                    "base_laser" + topic_postfix_, "base_footprint", ros::Time(0)
                );
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                return;
            }
            sensor_msgs::PointCloud2 cloud_transformed;
            tf2::doTransform(cloud, cloud_transformed, transform_stamped);

            // Timestamp and publish message
            cloud.header.frame_id = "base_footprint";
            cloud.header.stamp = scan_in -> header.stamp;
            cloud_pub_.publish(cloud);
            // cloud_transformed.header.stamp = scan_in -> header.stamp;
            // cloud_pub_.publish(cloud_transformed);
        }
};

int main(int argc, char** argv) {
  
  	// Setup the ROS node.
  	ros::init(argc, argv, "scan2cloud_node");
  	ros::NodeHandle nh;

  	ros::NodeHandle pr_nh("~"); // Private node handle to retrieve parameters

  	// Retrieve specified postfix, if any, to uniquely identify topics and transform frames
	std::string topic_postfix;
	if( !pr_nh.getParam("topic_postfix", topic_postfix) )
		topic_postfix = "";

	// Start up converter.
	LaserScanToPointCloud2 xv11_scan_lstopc(nh, 
		"xv11_scan" + topic_postfix,
		"base_footprint",
		"cloud" + topic_postfix,
        topic_postfix
	);
	ros::spin();

	return 0;
}