// cloud_cluster_metrics_node.cpp
// Splits an input point cloud into related clusters. For each cluster, computes the average and 
//     standard deviation of the Euclidean distance between nearest neighbors. These metrics are
//     used to control the LIDAR dithering.

// Cloud clusterization procedure used here is guided by the tutorial:
// 		http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php

// Neighbor search procedure used here guided by the tutorial:
// 		http://pointclouds.org/documentation/tutorials/kdtree_search.php

// Ryker Dial
// Date Created: January 18, 2018
// Last Modified: January 19, 2018

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <string>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

class CloudClusterMetrics {
	public:
		ros::NodeHandle nh_;
		ros::Subscriber cloud_sub_;
		ros::Publisher cloud_cluster_metrics_pub_;

		std::string cloud_topic_;

		pcl::SACSegmentation<pcl::PointXYZ> segmenter_;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster_extractor_;
  		pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree_;

		CloudClusterMetrics(ros::NodeHandle nh, std::string cloud_topic)
			: nh_(nh), cloud_topic_(cloud_topic), kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>) 
		{
			cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &CloudClusterMetrics::cloudCallback, this);

			// Set parameters of segmentation object for planar model
		  	segmenter_.setOptimizeCoefficients (true);
		  	segmenter_.setModelType(pcl::SACMODEL_PLANE);
		  	segmenter_.setMethodType(pcl::SAC_RANSAC);
		  	segmenter_.setMaxIterations(100);
		  	segmenter_.setDistanceThreshold(0.07);

		  	// Set parameters of euclidean cluster extractor
			euclidean_cluster_extractor_.setClusterTolerance(0.07);
			euclidean_cluster_extractor_.setMinClusterSize(100);
			euclidean_cluster_extractor_.setMaxClusterSize(25000);
			euclidean_cluster_extractor_.setSearchMethod(kd_tree_);

			cloud_cluster_metrics_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("cluster_spatial_stats", 1);
		}

		void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in) {
			// De-Serialize Point Cloud
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(*cloud_in, *cloud);

			// Compute metrics for each cluster
			ros::Time begin = ros::Time::now();
			std::vector<double> cluster_stats = obtainClusterMetrics(clusterizeCloud(cloud));
			ros::Time end = ros::Time::now();
			double elapsed_time = (end - begin).toSec();
			ROS_INFO("Computation of Cluster Metrics took %lf s", elapsed_time);

			// Create and configure ROS message
			std_msgs::Float64MultiArray cluster_spatial_stats_msg;
			cluster_spatial_stats_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
			cluster_spatial_stats_msg.layout.dim[0].size = cluster_stats.size();
			cluster_spatial_stats_msg.layout.dim[0].stride = 1;
			cluster_spatial_stats_msg.layout.dim[0].label = "cluster_stats";

			// Populate message
			for(int i=0; i<cluster_stats.size(); ++i) {
				cluster_spatial_stats_msg.data.push_back(cluster_stats[i]);
			}

			// Publish data
			cloud_cluster_metrics_pub_.publish(cluster_spatial_stats_msg);
		}

		std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clusterizeCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) {
			ros::Time begin = ros::Time::now();
			// Clusterize point cloud and extract cluster indices
			std::vector<pcl::PointIndices> cluster_indices;
			euclidean_cluster_extractor_.setInputCloud(cloud);
  			euclidean_cluster_extractor_.extract(cluster_indices);

  			// Create new clouds for each cluster.
  			std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
  			for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); 
  				it != cluster_indices.end(); ++it // For each cluster
  			) {
  				clusters.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>) );
  				for (std::vector<int>::const_iterator pit = it->indices.begin(); 
  					pit != it->indices.end (); ++pit // For each point in the cluster
  				) {
  					clusters.end()[-1] -> points.push_back(cloud -> points[*pit]);
					clusters.end()[-1] -> width = clusters.end()[-1] -> points.size();
					clusters.end()[-1] -> height = 1;
					clusters.end()[-1] -> is_dense = true;
  				}
  			}
  			ros::Time end = ros::Time::now();
  			double elapsed_time = (end-begin).toSec();
  			int num_points = 0;
  			for(size_t i=0; i<clusters.size(); ++i) {
  				num_points += clusters[i] -> points.size();
  			}
  			ROS_INFO("Identified %lu clusters representing a total of %d points. Took %lf s.",
  				clusters.size(), num_points, elapsed_time
  			);

  			return clusters;
		}

		std::vector<double> obtainClusterMetrics(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters) {
			std::vector<double> cluster_metrics;

			for(size_t i=0; i < clusters.size(); ++i) { // For each cluster
				double cluster_avg_var = 0; // Stores average euclidean distance for cluster
				double cluster_avg_var_numsamples = 0;

				double elapsed_time_avg = 0;
				//double num_neighbors_avg = 0;


				// Find the nearest neighbors within search radius
				pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
				kd_tree.setInputCloud(clusters[i]);

				for(size_t j=0; j < clusters[i] -> points.size(); ++j) { // For each point in the cluster
  					// TODO: Could this just be a random subset of points?
					ros::Time begin = ros::Time::now();

					// std::vector<int> pointIdxRadiusSearch; // Stores neighbor coordinates
  			// 		std::vector<float> pointRadiusSquaredDistance; // Stores distances to neighbors
  			// 		double radius = 0.07;
  					
  			// 		int num_neighbors = kd_tree.radiusSearch(
  			// 			clusters[i] -> points[j], // Point to search from
  			// 			radius,
  			// 			pointIdxRadiusSearch,
  			// 			pointRadiusSquaredDistance
  			// 		);
					int K = 10;
					std::vector<int> point_idx_NKN_search(K);
					std::vector<float> point_NKN_squared_distance(K);

					int num_neighbors = kd_tree.nearestKSearch(
						clusters[i] -> points[j], // Point to search from
						K,
						point_idx_NKN_search,
						point_NKN_squared_distance
					);

  					if(num_neighbors > 0) {
  						// Use Two-Pass algorithm to calculate variance
  						// Compute average squared euclidean distance
  						double avg_dist_sq = 0;
  						for(size_t k=0; k < num_neighbors; ++k) {
  							//avg_dist_sq += pointRadiusSquaredDistance[k];
  							avg_dist_sq += point_NKN_squared_distance[k];
  						}
  						avg_dist_sq = avg_dist_sq/num_neighbors;

  						// Compute variance of squared euclidean distance
  						double var_dist_sq = 0;
  						for(size_t k=0; k < num_neighbors; ++k) {
  							//var_dist_sq += (pointRadiusSquaredDistance[k] - avg_dist_sq)*(pointRadiusSquaredDistance[k] - avg_dist_sq);
  							var_dist_sq += (point_NKN_squared_distance[k] - avg_dist_sq)*(point_NKN_squared_distance[k] - avg_dist_sq);
  						}
  						var_dist_sq = var_dist_sq/num_neighbors;

  						// Add to overall cluster variance
  						cluster_avg_var += var_dist_sq;
  						++cluster_avg_var_numsamples;
  					}
  					ros::Time end = ros::Time::now();

  					elapsed_time_avg += (end-begin).toSec();
  					//num_neighbors_avg += num_neighbors;

  				}
  				elapsed_time_avg = elapsed_time_avg/(clusters[i] -> points.size());
  				//num_neighbors_avg = num_neighbors_avg/(clusters[i] -> points.size());

  				//ROS_INFO("Each point in cluster %lu had an average of %lf neighbors", i, num_neighbors_avg);
  				ROS_INFO("Finding neighbors and calculating variance for cluster %lu took on average %lf seconds.", i, elapsed_time_avg);

  				// Compute the average variance in euclidian distance for the entire cluster
  				cluster_avg_var = cluster_avg_var/cluster_avg_var_numsamples;
  				cluster_metrics.push_back(sqrt(cluster_avg_var)); // Add std dev to metrics
			}

			// Print out collected std devs
			for(size_t i=0; i < cluster_metrics.size(); ++i) {
				ROS_INFO("Average standard deviation of distance in cluster %lu: %lf", i, cluster_metrics[i]);
			}

			return cluster_metrics;
		}
};

int main(int argc, char** argv) {
	// Setup the ROS node.
	ros::init(argc, argv, "cloud_cluster_metrics_node");
	ros::NodeHandle nh;

	// Start up the clusterizer and statistics calculator
	CloudClusterMetrics cloud_cluster_metrics(nh, "points2");

	ros::spin();

	return 0;
}