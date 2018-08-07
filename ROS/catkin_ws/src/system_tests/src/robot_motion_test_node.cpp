#include <ros/ros.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <ncurses.h> // to get keyboard input.

ros::Subscriber odom_sub;
ros::Publisher cmd_vel_pub;
ros::Publisher acc_pub;

ros::Timer cmd_vel_timer;

geometry_msgs::Twist cmd_vel_msg;
std_msgs::Float32 acc_msg;

bool linear_test = false;
bool linear_test_done = false;
bool angular_test = false;
double test_length = 2.0; // Collect data for this long

std::vector<double> velocity_data;
std::vector<ros::Time> time_data;

ros::Time time_last_odom;
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
	time_last_odom = odom -> header.stamp;
	if(linear_test) {
		velocity_data.push_back(odom -> twist.twist.linear.x);
	}
	if(angular_test) {
		velocity_data.push_back(odom -> twist.twist.angular.z);
	}
	if(linear_test || angular_test) {
		time_data.push_back(time_last_odom);

		acc_msg.data = (velocity_data[velocity_data.size() - 1] - velocity_data[velocity_data.size() - 2])
						/((time_data[time_data.size() - 1] - time_data[time_data.size() - 2]).toSec());

		acc_pub.publish(acc_msg);
	}
}

void cmdVelTimerCallback(const ros::TimerEvent& event) {
	cmd_vel_pub.publish(cmd_vel_msg);
}

void linearTestCallback(const ros::TimerEvent & event) {
	linear_test = false;

	// Stop the robot's motion
	cmd_vel_msg.linear.x = 0;
	cmd_vel_msg.angular.z = 0;

	// Find maximum acceleration
	double max_acc = 0;
	for(int i=0; i<velocity_data.size()-1; ++i) {
		double acc = (velocity_data[i+1] - velocity_data[i])/((time_data[i+1]-time_data[i]).toSec());
		if( acc > max_acc) {
			max_acc = acc;
		}
	}
	// Print out acceleration
	printw("Max Linear Acceleration: %f\n\n", max_acc);
	refresh();

	// Clear vectors
	velocity_data.clear();
	time_data.clear();

	// Pause sending velocity commands to the robot
	cmd_vel_timer.stop();

	addstr("Press 's' once more to start the angular acceleration test.\n");
	refresh();
	linear_test_done = true;
}

void angularTestCallback(const ros::TimerEvent& event) {
	angular_test = false;

	// Stop the robot's motion
	cmd_vel_msg.linear.x = 0;
	cmd_vel_msg.angular.z = 0;

	// Find maximum acceleration
		double max_acc = 0;
		for(int i=0; i<velocity_data.size()-1; ++i) {
			double acc = (velocity_data[i+1] - velocity_data[i])/((time_data[i+1]-time_data[i]).toSec());
			if( acc > max_acc) {
				max_acc = acc;
			}
		}

	// Print out acceleration
	printw("Max Angular Acceleration: %f\n\n", max_acc);

	addstr("Acceleration Tests are complete. Press the spacebar to quit\n");
	refresh();
}

int main(int argc, char** argv) {
	// Setup the ROS node
	ros::init(argc, argv, "robot_motion_test_node");
	ros::NodeHandle nh;
	
	odom_sub = nh.subscribe("odom", 1, odomCallback);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	acc_pub = nh.advertise<std_msgs::Float32>("acc", 10);

	// Make sure command velocity is initialized to zero.
	cmd_vel_msg.linear.x = 0;
	cmd_vel_msg.angular.z = 0;

	// Timer for command velocity publisher
	cmd_vel_timer = nh.createTimer(ros::Duration(0.1), cmdVelTimerCallback);
	cmd_vel_timer.stop();

	// Timers for linear and angular acceleration tests. These are oneshot timer
	ros::Timer linear_test_timer = nh.createTimer(ros::Duration(test_length), linearTestCallback, true);
	linear_test_timer.stop();
	ros::Timer angular_test_timer = nh.createTimer(ros::Duration(test_length), angularTestCallback, true);
	angular_test_timer.stop();

	// Initialize ncurses
	initscr(); // Start curses
	raw(); // No line buffering
	noecho(); // Don't echo characters back to the terminal
	nodelay(stdscr, true); // Don't block waiting for user input

	// Print start messages
	addstr("This node will estimate the max acceleration of the robot using the wheel odometry.\n");
	addstr("Before test, make sure robot has plenty of room to move around.\n");
	addstr("Press the 's' key to begin. Press spacebar at any time to abort.\n\n");
	refresh();

	// Wait for start or abort command
	char key;
	do {
		key = getch();
	} while((key != 's') && (key != ' '));

	if(key == ' ') {
		endwin();
		return 0;
	}

	// Begin the test
	addstr("Starting tests.\n");
	addstr("Beginning with linear acceleration test.\n");
	refresh();

	// Begin Linear Acceleration Test
	ros::spinOnce(); // Clear all callbacks from queue.
	cmd_vel_msg.linear.x = 0.5; // Max create 2 linear velocity as specified by create_autonomy package
	velocity_data.push_back(0); // Starting from rest
	time_data.push_back(time_last_odom); // Put starting time at last odom update.
	cmd_vel_pub.publish(cmd_vel_msg);
	linear_test = true;
	ros::spinOnce(); // Send cmd velocity 
	cmd_vel_timer.start();
	linear_test_timer.start();

	while(ros::ok()) {
		ros::spinOnce();

		key = getch();
		if(key == ' ') {
			// Spacebar pressed. Abort.
			cmd_vel_timer.stop();

			// Send power down command
			cmd_vel_msg.linear.x = 0;
			cmd_vel_msg.angular.z = 0;
			cmd_vel_pub.publish(cmd_vel_msg);
			ros::spinOnce();

			break;
		}

		// If we've finished the linear test, perform the angular test.
		if(key == 's' && linear_test_done) {
			linear_test_done = false;

			cmd_vel_msg.linear.x = 0.5;
			cmd_vel_msg.angular.z = 4.25; // Max create 2 linear velocity as specified by create_autonomy package
			velocity_data.push_back(0); // Starting from rest
			time_data.push_back(time_last_odom); // Put starting time at last odom update.
			cmd_vel_pub.publish(cmd_vel_msg);
			angular_test = true;
			ros::spinOnce(); // Send cmd velocity 
			cmd_vel_timer.start();
			angular_test_timer.start();
		}

	}

	endwin(); // End curses
	return 0;
}