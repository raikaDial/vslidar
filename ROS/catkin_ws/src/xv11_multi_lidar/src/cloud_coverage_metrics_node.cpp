// cloud_coverage_metrics_node.cpp

// Ryker Dial
// Date Created: February 20, 2018
// Last Modified: February 28, 2018

// ROS Libraries
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

// PCL Libraries
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <string>
#include <sstream> // To concatenate strings and ints, because C++11 results in segfault...
#include <cmath>

// Returns the ratio of points in the cloud before and after voxel filtering, as a measure
//     of unique data.
class VoxelMetric {
	public:
		VoxelMetric(ros::NodeHandle nh) : nh_(nh) {
			// Get voxel resolution parameter
			ros::NodeHandle pr_nh("~");
			pr_nh.param<double>("voxel_resolution", voxel_resolution_, 0.1);
			pr_nh.param<bool>("publish_voxel_cloud", publish_voxel_cloud_, false);

			// Configure voxel grid filter
			sor_.setLeafSize(voxel_resolution_, voxel_resolution_, voxel_resolution_);

			if(publish_voxel_cloud_) {
				cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("voxel_metric_cloud", 10);
			}		
		}

		double computeMetric(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in) {
			// Compute coverage ratio and return
			sor_.setInputCloud(cloud_in);
			int points_before_filter = (cloud_in -> width) * (cloud_in -> height);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
			sor_.filter(*cloud_filtered);
			int points_after_filter = (cloud_filtered -> width) * (cloud_filtered -> height);

			if(publish_voxel_cloud_) {
				sensor_msgs::PointCloud2 cloud_msg;
				pcl::toROSMsg(*cloud_filtered, cloud_msg);
				cloud_msg.header.frame_id = "odom";
				cloud_msg.header.stamp = ros::Time::now();
				cloud_pub_.publish(cloud_msg);
			}

			return ((double) points_after_filter)/((double) points_before_filter);
		}

	private:
		// Voxel grid filter for computing cloud coverage metric
		pcl::VoxelGrid<pcl::PointXYZ> sor_;
		double voxel_resolution_;

		// Voxel Cloud Publisher for Debugging
		ros::NodeHandle nh_;
		ros::Publisher cloud_pub_;
		bool publish_voxel_cloud_;
};

class DistStdDevMetric {
	public:
		DistStdDevMetric() : kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>) {
			// Set parameters of segmentation object for planar model
		  	segmenter_.setOptimizeCoefficients (true);
		  	segmenter_.setModelType(pcl::SACMODEL_PLANE);
		  	segmenter_.setMethodType(pcl::SAC_RANSAC);
		  	segmenter_.setMaxIterations(100);
		  	segmenter_.setDistanceThreshold(0.15);

		  	// Set parameters of euclidean cluster extractor
			euclidean_cluster_extractor_.setClusterTolerance(0.15);
			euclidean_cluster_extractor_.setMinClusterSize(100);
			euclidean_cluster_extractor_.setMaxClusterSize(25000);
			euclidean_cluster_extractor_.setSearchMethod(kd_tree_);
		}

		double computeMetric(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in) {
			ros::Time begin = ros::Time::now();

			// Compute metrics for each cluster
			std::vector<double> cluster_stats = obtainClusterMetrics(clusterizeCloud(cloud_in));

			// Find average across all clusters
			double avg_std_dev = 0;
			for(int i=0; i<cluster_stats.size(); ++i) {
				avg_std_dev += cluster_stats[i];
			}
			avg_std_dev /= cluster_stats.size();

			ros::Time end = ros::Time::now();
			double elapsed_time = (end - begin).toSec();
			ROS_INFO("Average Std. Dev. in Distance for All Clusters: %lf\n", avg_std_dev);
			ROS_INFO("Computation of Cluster Metrics took %lf s", elapsed_time);

			return avg_std_dev;
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

					std::vector<int> pointIdxRadiusSearch; // Stores neighbor coordinates
  					std::vector<float> pointRadiusSquaredDistance; // Stores distances to neighbors
  					double radius = 0.2;
  					
  					int num_neighbors = kd_tree.radiusSearch(
  						clusters[i] -> points[j], // Point to search from
  						radius,
  						pointIdxRadiusSearch,
  						pointRadiusSquaredDistance
  					);
					// int K = 10;
					// std::vector<int> point_idx_NKN_search(K);
					// std::vector<float> point_NKN_squared_distance(K);

					// int num_neighbors = kd_tree.nearestKSearch(
					// 	clusters[i] -> points[j], // Point to search from
					// 	K,
					// 	point_idx_NKN_search,
					// 	point_NKN_squared_distance
					// );

  					if(num_neighbors > 0) {
  						// Use Two-Pass algorithm to calculate variance
  						// Compute average squared euclidean distance
  						double avg_dist_sq = 0;
  						for(size_t k=0; k < num_neighbors; ++k) {
  							avg_dist_sq += pointRadiusSquaredDistance[k];
  							//avg_dist_sq += point_NKN_squared_distance[k];
  						}
  						avg_dist_sq = avg_dist_sq/num_neighbors;

  						// Compute variance of squared euclidean distance
  						double var_dist_sq = 0;
  						for(size_t k=0; k < num_neighbors; ++k) {
  							var_dist_sq += (pointRadiusSquaredDistance[k] - avg_dist_sq)*(pointRadiusSquaredDistance[k] - avg_dist_sq);
  							//var_dist_sq += (point_NKN_squared_distance[k] - avg_dist_sq)*(point_NKN_squared_distance[k] - avg_dist_sq);
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

	private:
		pcl::SACSegmentation<pcl::PointXYZ> segmenter_;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster_extractor_;
  		pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree_;
};

// This class extracts planar components from the input point cloud and computes the 2D coverage
// TODO: Make this work with planes of arbitrary orientation
class PlanarCoverageMetric {
	public:
		PlanarCoverageMetric(ros::NodeHandle nh)
		: nh_(nh), coefficients_(new pcl::ModelCoefficients), inliers_(new pcl::PointIndices), iteration_(0) {

			// Get parameters
			ros::NodeHandle pr_nh("~");
			pr_nh.param<double>("coverage_grid_resolution", coverage_grid_resolution_, 0.1);
			pr_nh.param<bool>("publish_coverage_visuals", publish_coverage_visuals_, false);
			pr_nh.param<int>("lidar_ID", lidar_ID_, 1);
			std::stringstream oss;
			oss << lidar_ID_;
			std::string lidar_ID_str = oss.str();

			// If configured, publish coverage map and associated visuals.
			if(publish_coverage_visuals_) {
				cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("planar_cloud_" + lidar_ID_str, 1);
				cloud_bounding_box_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("planar_cloud_bounding_box_" + lidar_ID_str, 1);
				coverage_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("coverage_map_" + lidar_ID_str, 1);
			}

			// Configure the planar segmenter
			segmenter_.setOptimizeCoefficients(true);
			segmenter_.setModelType(pcl::SACMODEL_PLANE);
			segmenter_.setMethodType(pcl::SAC_RANSAC);
			segmenter_.setMaxIterations(1000);
			segmenter_.setDistanceThreshold(0.01);

			// Configure statistical outlier removal filter
			// TODO: Tune these for optimal performance
			sor_.setMeanK(50);
			sor_.setStddevMulThresh(1.0);
		}

		// Determines whether or not a point is contained within the provided convex polygon
		bool isInHull(pcl::PointXYZ point, const geometry_msgs::PolygonStamped & boundary) {
			// Loop counterclockwise over all edges of the hull. If point is to the left of all edges,
			//     then it is within the hull.
			// Sources:
			//     https://stackoverflow.com/questions/16750618/whats-an-efficient-way-to-find-if-a-point-lies-in-the-convex-hull-of-a-point-cl
			//     https://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line
			size_t num_vertices = boundary.polygon.points.size();

			for(int i=0; i<num_vertices; ++i) {
				double Ax = boundary.polygon.points[i].x;
				double Ay = boundary.polygon.points[i].y;
				double Bx = boundary.polygon.points[(i+1)%num_vertices].x;
				double By = boundary.polygon.points[(i+1)%num_vertices].y;

				// Position is < 0 if the point is outside the boundary
				double position = (Bx - Ax)*(point.y - Ay) - (By - Ay)*(point.x - Ax);
				if(position >= 0) {
					return false;
				}
			}
			return true;
		}

		// Computes the ratio occupied_cells/total_cells, where the cells are within the convex hull of the largest planar
		//     component of the given cloud
		double computeMetric(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in) {
			ros::Time begin = ros::Time::now();
			// Identify the largest planar component from the cloud
			segmenter_.setInputCloud(cloud_in);
			segmenter_.segment(*inliers_, *coefficients_);

			if(inliers_->indices.size() == 0) {
				ROS_WARN("No planar component found in the accumulated cloud.");
				return 0;
			}

			// Extract the planar component
			pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			extract_.setInputCloud(cloud_in);
			extract_.setIndices(inliers_);
			extract_.setNegative(false);
			extract_.filter(*planar_cloud);

			ROS_INFO("Points representing planar component: %u\n", planar_cloud -> width * planar_cloud -> height);
			ROS_INFO("Model Coefficients, %f, %f, %f, %f\n",
				coefficients_ -> values[0], coefficients_ -> values[1], coefficients_ -> values[2], coefficients_ -> values[3] 
			);

			// Remove outliers
			pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
			sor_.setInputCloud(planar_cloud);
			sor_.filter(*planar_cloud_filtered);

			// Rotate the plane so it is parallel to the x-y plane
			// Source: 
			//     https://stackoverflow.com/questions/32299903/rotation-and-transformation-of-a-plane-to-xy-plane-and-origin-in-pointcloud
			
			// First, construct the normal vectors for the XY plane and the planar cloud
			Eigen::Matrix<float, 1, 3> xy_normal, plane_normal, rotation_vector;
			xy_normal[0] = 0.0;
			xy_normal[1] = 0.0;
			xy_normal[2] = 1.0;
			plane_normal[0] = coefficients_ -> values[0];
			plane_normal[1] = coefficients_ -> values[1];
			plane_normal[2] = coefficients_ -> values[2];

			// Calculate the rotation direction and rotation angle
			rotation_vector = xy_normal.cross(plane_normal);
			rotation_vector.normalize();
			double rotation_angle = -acos(plane_normal.dot(xy_normal));

			// Transform the point cloud
			Eigen::Affine3f planar_cloud_transform = Eigen::Affine3f::Identity();
			planar_cloud_transform.rotate(Eigen::AngleAxisf(rotation_angle, rotation_vector));
			pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud_rotated (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::transformPointCloud(*planar_cloud_filtered, *planar_cloud_rotated, planar_cloud_transform);


			// Compute the convex hull of the point cloud for a bounding polygon
			pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
			chull_.setInputCloud(planar_cloud_rotated);
			chull_.setDimension(2); // 2D
			std::vector<pcl::Vertices> bounding_polygon;
			chull_.reconstruct(*planar_cloud_hull, bounding_polygon);

			// Extract the coordinates for the covex hull
			geometry_msgs::PolygonStamped planar_cloud_boundary;
			planar_cloud_boundary.header.stamp = ros::Time::now();
			planar_cloud_boundary.header.frame_id = "base_footprint";
			geometry_msgs::Point32 pt;
			for(int i=0; i<bounding_polygon[0].vertices.size(); ++i) {
				pt.x = (*planar_cloud_hull).at(bounding_polygon[0].vertices[i]).x;
				pt.y = (*planar_cloud_hull).at(bounding_polygon[0].vertices[i]).y;
				pt.z = (*planar_cloud_hull).at(bounding_polygon[0].vertices[i]).z;
				planar_cloud_boundary.polygon.points.push_back(pt);
			}

			// Get a bounding box for the cloud for grid dimensions
			pcl::PointXYZ min_pt, max_pt;
			pcl::getMinMax3D(*planar_cloud_rotated, min_pt, max_pt);

			// Initialize the grid
			double grid_x = max_pt.x - min_pt.x;
			double num_cells_x = ceil(grid_x/coverage_grid_resolution_);
			double grid_y = max_pt.y - min_pt.y;
			double num_cells_y = ceil(grid_y/coverage_grid_resolution_);
			
			std::vector< std::vector<int8_t> > coverage_grid (num_cells_x, std::vector<int8_t> (num_cells_y, 0));

			// Populate the grid. Start with points contained in the point cloud.
			size_t num_occupied_cells = 0;
			for(pcl::PointCloud<pcl::PointXYZ>::iterator it=planar_cloud_rotated->begin(); it!=planar_cloud_rotated->end(); ++it) {
				size_t x_coord = (it -> x - min_pt.x)/coverage_grid_resolution_;
				size_t y_coord = (it -> y - min_pt.y)/coverage_grid_resolution_;

				// Clamp coordinates to ensure they are within the grid
				if(x_coord > (num_cells_x-1)) x_coord = num_cells_x-1;
				if(y_coord > (num_cells_y-1)) y_coord = num_cells_y-1;

				// Set occupied space equal to 100
				if(coverage_grid[x_coord][y_coord] != 100) {
					coverage_grid[x_coord][y_coord] = 100;
					++num_occupied_cells;
				}
			}
			size_t num_cells_in_hull = num_occupied_cells;

			size_t num_empty_cells = 0;
			// Now, iterate through the rest of the cells. If an empty cell lies within the convex hull, set it equal to 50.
			for(size_t y=0; y<num_cells_y; ++y) {
				for(size_t x=0; x<num_cells_x; ++x) {
					if(coverage_grid[x][y] != 100) { // Cell is not occupied
						pcl::PointXYZ point;
						point.x = x*coverage_grid_resolution_ + min_pt.x;
						point.y = y*coverage_grid_resolution_ + min_pt.y;

						if(isInHull(point, planar_cloud_boundary)) {
							coverage_grid[x][y] = 50;
							++num_empty_cells;
						}
					}
				}
			}
			num_cells_in_hull += num_empty_cells;
			double cloud_coverage = ((double)num_occupied_cells) / ((double)num_cells_in_hull);

			ros::Time end = ros::Time::now();
			ROS_INFO("Computation of planar metrics took %lf seconds.\n", (end-begin).toSec());
			ROS_INFO("Cloud coverage of largest planar component is %f%%.\n", cloud_coverage*100);

			++iteration_;
			// Visualize coverage map as an occupancy grid
			if(publish_coverage_visuals_) {
				nav_msgs::OccupancyGrid coverage_grid_msg;
				coverage_grid_msg.header.stamp = ros::Time::now();
				coverage_grid_msg.header.frame_id = "base_footprint";
				coverage_grid_msg.header.seq = iteration_;
				coverage_grid_msg.info.resolution = coverage_grid_resolution_;
				coverage_grid_msg.info.width = num_cells_x;
				coverage_grid_msg.info.height = num_cells_y;
				coverage_grid_msg.info.origin.position.x = min_pt.x;
				coverage_grid_msg.info.origin.position.y = min_pt.y;

				coverage_grid_msg.data.reserve(num_cells_x*num_cells_y);
				for(int i=0; i<num_cells_y; ++i) {
					for(int j=0; j<num_cells_x; ++j) {
						coverage_grid_msg.data.push_back(coverage_grid[j][i]);
					}
				}

				coverage_grid_pub_.publish(coverage_grid_msg);

				// Publish the planar cloud
				sensor_msgs::PointCloud2 cloud_msg;
				pcl::toROSMsg(*planar_cloud_rotated, cloud_msg);
				cloud_msg.header.frame_id = "base_footprint";
				cloud_msg.header.stamp = ros::Time::now();
				cloud_msg.header.seq = iteration_;
				cloud_pub_.publish(cloud_msg);

				// Publish the bounding polygon
				planar_cloud_boundary.header.seq = iteration_;
				cloud_bounding_box_pub_.publish(planar_cloud_boundary);
			}

			return cloud_coverage;
		}	


	private:
		ros::NodeHandle nh_;
		ros::Publisher cloud_pub_;
		ros::Publisher cloud_bounding_box_pub_;
		ros::Publisher coverage_grid_pub_;

		double coverage_grid_resolution_;
		bool publish_coverage_visuals_;
		int lidar_ID_;
		size_t iteration_;

		pcl::ModelCoefficients::Ptr coefficients_;
		pcl::PointIndices::Ptr inliers_;
		pcl::SACSegmentation<pcl::PointXYZ> segmenter_;
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_;
		pcl::ConvexHull<pcl::PointXYZ> chull_;
		pcl::ExtractIndices<pcl::PointXYZ> extract_;
};

class CloudCoverageMetrics {
	public:
		CloudCoverageMetrics(ros::NodeHandle nh) 
		: nh_(nh), listener_(tf_buffer_), cloud_accumulated_current_(new pcl::PointCloud<pcl::PointXYZ>), 
		  cloud_accumulated_previous_(new pcl::PointCloud<pcl::PointXYZ>), voxel_metric_(nh), planar_metric_(nh)
		{
			// Get parameters
			ros::NodeHandle pr_nh("~");
			//pr_nh.param<double>("publish_period", publish_period_, 4.0);
			pr_nh.param<bool>("use_voxel_metric", use_voxel_metric_, false);
			pr_nh.param<bool>("use_dist_std_dev_metric", use_dist_std_dev_metric_, false);
			pr_nh.param<bool>("publish_accumulated_cloud", publish_accumulated_cloud_, false);
			pr_nh.param<bool>("use_planar_metric", use_planar_metric_, true);
			pr_nh.param<int>("lidar_ID", lidar_ID_, 1);
			std::stringstream oss;
			oss << lidar_ID_;
			std::string lidar_ID_str = oss.str();
			
			// init_pan_params_ = true;
			// timer_active = false;

			// Cloud topic is published at a rate of about 75 Hz.
			// Use a rolling cloud buffer to determine effects of dithering
			// double cloud_buffer_temporal_length;
			// pr_nh.param<double>("cloud_buffer_temporal_length", cloud_buffer_temporal_length, 6.0);
			// cloud_buffer_size_ = 75*cloud_buffer_temporal_length;

			// Start up publishers and subscribers
			cloud_sub_ = nh_.subscribe("cloud_" + lidar_ID_str, 10, &CloudCoverageMetrics::cloudCallback, this);
			ax12_done_panning_sub_ = nh_.subscribe("ax12_done_panning_" + lidar_ID_str, 1, &CloudCoverageMetrics::donePanningCallback, this);
			//pan_param_sub_ = nh_.subscribe("configured_pan_params", 1, &CloudCoverageMetrics::panParamCallback, this);
			cloud_coverage_metrics_pub_ = nh_.advertise<std_msgs::Float64>("cloud_coverage_metric_" + lidar_ID_str, 1);

			if(publish_accumulated_cloud_) {
				cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_accumulated_" + lidar_ID_str, 1);
			}

			// // Startup metric publisher timer
			// metric_publish_timer_ = nh_.createTimer(ros::Duration(publish_period_), &CloudCoverageMetrics::timerCallback, this, true);
		}

		void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in) {
			// Transform the cloud's coordinate frame from base_footprint to odom.
            geometry_msgs::TransformStamped transform_stamped;
			try {
				transform_stamped = tf_buffer_.lookupTransform(
					"odom", "base_footprint", ros::Time(0)
				);
			}
			catch (tf2::TransformException &ex) {
	   			ROS_WARN("%s",ex.what());
	   			return;
	   		}
	   		sensor_msgs::PointCloud2 cloud_transformed;
	   		tf2::doTransform(*cloud_in, cloud_transformed, transform_stamped);
	   		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_buff(new pcl::PointCloud<pcl::PointXYZ>);
	   		pcl::fromROSMsg(cloud_transformed, *cloud_buff);
	   		*cloud_accumulated_current_ += *cloud_buff;
		}

		void donePanningCallback(const std_msgs::Empty::ConstPtr& msg) {
			if(!(cloud_accumulated_previous_ -> empty())) { // If we have enough data
				// Combine point clouds from previous and current revolutions
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_accumulated(new pcl::PointCloud<pcl::PointXYZ>);
				*cloud_accumulated = *cloud_accumulated_previous_ + *cloud_accumulated_current_;

				// Convert back to base_footprint coordinates
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
		   		sensor_msgs::PointCloud2 cloud_buff;
				pcl::toROSMsg(*cloud_accumulated, cloud_buff);
				tf2::doTransform(cloud_buff, cloud_buff, transform_stamped);
				pcl::fromROSMsg(cloud_buff, *cloud_accumulated);

				// Compute and publish cloud coverage metric
				std_msgs::Float64 coverage_metric_msg;
				coverage_metric_msg.data = computeCoverageMetric(cloud_accumulated);
				cloud_coverage_metrics_pub_.publish(coverage_metric_msg);

				if(publish_accumulated_cloud_) {
					sensor_msgs::PointCloud2 cloud_msg;
					pcl::toROSMsg(*cloud_accumulated, cloud_msg);
					cloud_msg.header.frame_id = "odom";
					cloud_msg.header.stamp = ros::Time::now();
					cloud_pub_.publish(cloud_msg);

				}
			}
			// Store current cloud in previous cloud and clear current cloud for next cycle
			cloud_accumulated_previous_ -> swap(*cloud_accumulated_current_);
			cloud_accumulated_current_ -> clear();			
		}

		double computeCoverageMetric(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_accumulated) {
			// Compute the user specified metric
			double metric = 0;
			if(use_voxel_metric_) {
				metric = voxel_metric_.computeMetric(cloud_accumulated);
			}
			else if(use_dist_std_dev_metric_) {

				metric = dist_std_dev_metric_.computeMetric(cloud_accumulated);
			}
			else if(use_planar_metric_) {
				metric = planar_metric_.computeMetric(cloud_accumulated);
			}
			else {
				ROS_WARN("All coverage metrics set to false!\n");
			}
			return metric;
		}

	private:
		ros::NodeHandle nh_;
		ros::Subscriber cloud_sub_;
		ros::Subscriber ax12_done_panning_sub_;
		ros::Publisher cloud_pub_;
		ros::Publisher cloud_coverage_metrics_pub_;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_accumulated_previous_;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_accumulated_current_;

		int lidar_ID_; // Which LIDAR to publish metrics for

		tf2_ros::TransformListener listener_;
		tf2_ros::Buffer tf_buffer_; // Buffer stores received transforms

		// Various Metrics
		VoxelMetric voxel_metric_;
		DistStdDevMetric dist_std_dev_metric_;
		PlanarCoverageMetric planar_metric_;

		bool use_voxel_metric_;
		bool use_dist_std_dev_metric_;
		bool use_planar_metric_;
		bool publish_accumulated_cloud_;
};

int main(int argc, char** argv) {
	// Setup the ROS node.
	ros::init(argc, argv, "cloud_coverage_metrics_node");
	ros::NodeHandle nh;

	// Start up the clusterizer and statistics calculator
	CloudCoverageMetrics cloud_coverage_metrics(nh);

	ros::spin();

	return 0;
}