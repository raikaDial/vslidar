// var_scan_test_node.cpp

// Ryker Dial
// Date Created: January 16, 2018
// Last Modified: January 16, 2018

#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::Publisher pan_param_pub;
std_msgs::Int16MultiArray pan_param_msg;

bool toggle_fast = true;
ros::Duration toggle_period = ros::Duration(1.0);
ros::Duration wait_time = ros::Duration(0.1);

void timer_cb(const ros::TimerEvent&) {
	if(toggle_fast) {
		for(int i=2; i<=2; ++i) {
			pan_param_msg.data.clear();
			pan_param_msg.data.push_back(i); // Set ID
			pan_param_msg.data.push_back(-1); // CW Limit (Leave unchanged)
			pan_param_msg.data.push_back(-1); // CCW Limit (Leave unchanged)
			pan_param_msg.data.push_back((int)(30)); // rotation step scalar, 0 to 1023;
			pan_param_pub.publish(pan_param_msg);

			if(i != 2) {
				wait_time.sleep();
			}
		}
	}
	// else {
	// 	for(int i=1; i<=2; ++i) {
	// 		pan_param_msg.data.clear();
	// 		pan_param_msg.data.push_back(i); // Set ID
	// 		pan_param_msg.data.push_back(-1); // CW Limit (Leave unchanged)
	// 		pan_param_msg.data.push_back(-1); // CCW Limit (Leave unchanged)
	// 		pan_param_msg.data.push_back((int)(0.25*1023));
	// 		pan_param_pub.publish(pan_param_msg);

	// 		if(i != 2) {
	// 			wait_time.sleep();
	// 		}
	// 	}
	// }
	toggle_fast = !toggle_fast;
}

int main(int argc, char** argv) {
	// Setup the ROS node
	ros::init(argc, argv, "var_scan_test_node");
	ros::NodeHandle nh;

	// Setup scan rate publisher and parameter message
	pan_param_pub = nh.advertise<std_msgs::Int16MultiArray>("pan_params", 10);
	pan_param_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	pan_param_msg.layout.dim[0].size = 4;
	pan_param_msg.layout.dim[0].stride = 1;
	pan_param_msg.layout.dim[0].label = "pan_params";

	ros::Timer pan_toggle_timer = nh.createTimer(toggle_period, timer_cb);

	ros::spin();

	return 0;
}