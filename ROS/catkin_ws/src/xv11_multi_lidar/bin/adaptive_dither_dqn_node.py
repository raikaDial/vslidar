#! /usr/bin/env python

# adaptive_dither_dqn_node.py
# Adjusts LIDAR sweeping rates to minimize standard deviation in Euclidean
# 	  distances between point in obtained clouds, adaptively dithering the
#	  sensors to improve uniformity in point distribution and intelligently
# 	  increase sensor coverage area.

# DNN implemented with guidance of tutorial:
#     https://towardsdatascience.com/reinforcement-learning-w-keras-openai-dqns-1eed3a5338c

# Ryker Dial
# Date Created: January 19, 2018
# Last Modified: March 5, 2018

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32 
from std_msgs.msg import Float64 
from std_msgs.msg import Int16MultiArray # For panning parameters
from nav_msgs.msg import Odometry # For filtered odometry

import dynamic_reconfigure.client

import numpy
import random
from keras.models import Sequential
from keras.layers import Dense, Dropout
from keras.optimizers import Adam

from collections import deque
import itertools

class VslidarEnvironment:
	def __init__(self, lidar_ID):
		### Initialize Environment State Space ###

		# These store info on the robot's and LIDAR's motion in-between state updates
		self.linear_velocity = []
		self.angular_velocity = []
		self.lidar_pan_speed = -1
		self.servo_angle = -1

		self.state_shape = 5 

		self.lidar_ID = lidar_ID; # Which LIDAR to keep track of

		######

		### Initialize Environment Actions Space #####

		# Allowable speeds for each LIDAR, in % of max speed
		self.action_space = [0.025, 0.05, 0.075, 0.10, 0.125, 0.15, 0.175, 0.20, 0.225, 0.25]
		self.action_shape = len(self.action_space)

		self.max_linear_velocity = rospy.get_param('/move_base/TrajectoryPlannerROS/max_vel_x', 0.25)

		######

		# Stores complete state information
		self.memory = deque(maxlen=2000)
		######

		# Initialize ROS publisher to perform actions
		self.pan_param_pub = rospy.Publisher('pan_params', Int16MultiArray, queue_size = 10)

		# Initialize ROS subscribers to obtain state information
		self.odom_sub = rospy.Subscriber("odometry/filtered", Odometry, self.odometryCallback, queue_size = 10)
		self.configured_pan_params_sub = rospy.Subscriber("configured_pan_params", Int16MultiArray, self.configuredPanParamsCallback, queue_size = 10)
		self.servo_angle_sub = rospy.Subscriber("ax12_angle_" + str(self.lidar_ID), Float32, self.angleCallback, queue_size = 10)
		
	def odometryCallback(self, odom):
		# Store robot velocities
		self.linear_velocity.append(odom.twist.twist.linear.x)
		self.angular_velocity.append(odom.twist.twist.angular.z)

	def configuredPanParamsCallback(self, params):
		if params.data[0] == self.lidar_ID:
			self.lidar_pan_speed = params.data[3]/1023.0

	def angleCallback(self, angle):
		self.servo_angle = angle.data

	def update(self, metric):
		# Store the current state
		# Weird array nesting for Keras compatibility
		self.memory.append(
			numpy.array([[
				numpy.mean(self.linear_velocity),
				numpy.mean(self.angular_velocity),
				self.lidar_pan_speed,
				self.servo_angle,
				metric
			]])
		)

		# Clear velocity buffers
		self.linear_velocity[:] = []
		self.angular_velocity[:] = []

	def performAction(self, action):
		# Set LIDAR to pan at specified speed
		pan_param_msg = Int16MultiArray(data=[self.lidar_ID, -1, -1, self.action_space[action]*1023])
		self.pan_param_pub.publish(pan_param_msg)
		self.lidar_pan_speed = self.action_space[action];

		# Scale driving speed so the robot drives more slowly when it is panning more slowly
		# TODO: Make it so two dithering nodes can be active and coordinate which one controls the driving
		linear_velocity_limit_new = self.max_linear_velocity*(0.5+2*self.action_space[action])
		client = dynamic_reconfigure.client.Client('move_base/TrajectoryPlannerROS')
		params = {'max_vel_x' : linear_velocity_limit_new}
		config = client.update_configuration(params)

		rospy.loginfo("Set LIDAR %s to %s%% Speed\n", self.lidar_ID, self.action_space[action]*100)
		rospy.loginfo('Set max autonomous driving speed to %s m/s.\n', linear_velocity_limit_new)

		return

	# Returns the reward for the previously taken action
	def getReward(self):
		coverage = self.memory[-1][0][4]
		pan_velocity = self.memory[-1][0][2]
		# coverage = self.memory[-1][0][6]

		# # Compute the average combined LIDAR velocity for over the specified number of previous samples
		# avg_combined_lidar_velocity = 0
		# num_samples = min(len(self.memory), self.num_speeds_avg)
		# for i in range(0, num_samples-1):
		# 	avg_combined_lidar_velocity += self.memory[-1-i][0][2] + self.memory[-1-i][0][3]
		# avg_combined_lidar_velocity /= num_samples

		# reward = ((coverage ** 1.5) * ((avg_combined_lidar_velocity) ** (0.5 / coverage))) / 0.70711
		reward = 2*((coverage ** 1.5) * ((pan_velocity) ** (0.5)))
		# reward = coverage ** 1.5
		return reward


class ScanDitherDQN:
	def __init__(self):
		# Seed random number generator for reproducibility
		numpy.random.seed(42)

		lidar_ID = rospy.get_param('~lidar_ID', 1)

		# Create an instance of the environment
		self.env = VslidarEnvironment(lidar_ID)

		# Discount to give to predicted future rewards. This number should be higher for deterministic
		#     environments and lower for stochastic environments
		self.gamma = 0.95

		# Epsilon and its decay rate dictates how much time the network will spend exploring
		#     new strategies vs. applying existing strategies (exploration vs. exploitation).
		self.epsilon = 1.0
		self.epsilon_min = 0.01
		self.epsilon_decay = 0.995

		self.learning_rate = 0.01 # The rate at which information accumulates in the neural net between epochs

		# ANN memory stores the previous state, the action taken, the reward, the new state, 
		#     and whether we are done collecting data
		self.memory = deque(maxlen=2000)

		# Create two neural nets,.
		# model performs predictions on what actions to take
		# target_model tracks what action we want our model to take
		load_previous_model = rospy.get_param('~load_previous_model', False)
		self.model_filename = rospy.get_param('~model_filename', 'dither_dqn_model.h5')
		if load_previous_model:
			self.model = load_model(model_filename)
			self.target_model = load_model(model_filename)
		else:
			self.model = self.create_model()
			self.target_model = self.create_model()

		# If our model is already trained, lower the explore/exploit epsilon to the minimum value
		model_is_trained = rospy.get_param('~model_is_trained', False)
		if model_is_trained:
			self.epsilon = self.epsilon_min

		self.tau = 15 # How many iterations in between target model update
		#0.125 # Controls integration of prediction weights with target weights for target model

		self.iterations = 0 # Keep track of how many iterations we have gone through

		self.ready_to_update = False

		# Subscribe to the cloud coverage metric. Publish rate controls step rate of neural network
		self.coverage_metric_sub = rospy.Subscriber("cloud_coverage_metric_" + str(lidar_ID), Float64, self.coverageMetricCallback, queue_size = 1)

	# Creates the neural network with the parameters specified in the constructor
	def create_model(self):
		model = Sequential()

		# Use relu activation units because they have better performance
		model.add(Dense(self.env.state_shape, input_dim = self.env.state_shape, activation = "relu"))
		model.add(Dense(48, activation = "relu"))
		model.add(Dense(24, activation = "relu"))

		# Q-Value is sum total of all future rewards, so use linear activation function
		model.add(Dense(self.env.action_shape, activation = "linear"))

		model.compile(optimizer = Adam(lr = self.learning_rate), loss = "mean_squared_error")

		return model

	def remember(self, state, action, reward, new_state, done):
		self.memory.append([state, action, reward, new_state, done])
		rospy.loginfo("Previous action: %s, reward: %s", self.env.action_space[action], reward)

	def replay(self):
		batch_size = 32
		if len(self.memory) < batch_size:
			return

		samples = random.sample(numpy.array(self.memory), batch_size)
		for sample in samples:
			state, action, reward, new_state, done = sample
			target = self.target_model.predict(state)

			if done:
				target[0][action] = reward
			else:
				Q_future = max(self.target_model.predict(new_state)[0])
				target[0][action] = reward + Q_future*self.gamma

			self.model.fit(state, target, epochs=1, verbose=0)

	# Train our target model by copying the weights from the main model
	def target_train(self):
		weights = self.model.get_weights()
		target_weights = self.target_model.get_weights()
		for i in range(len(target_weights)):
			target_weights[i] = weights[i]*self.tau + target_weights[i]*(1-self.tau)
		self.target_model.set_weights(target_weights)

	def act(self, state):
		# Decay epsilon
		self.epsilon *= self.epsilon_decay
		self.epsilon = max(self.epsilon_min, self.epsilon)

		# Randomly explore or exploit
		# Returns the index to the action in the action_space array
		if numpy.random.random() < self.epsilon:
			rospy.loginfo("Performing a random action.\n")
			return random.randint(0, self.env.action_shape - 1)
		rospy.loginfo("Performing a predicted action.\n")
		return numpy.argmax(self.model.predict(state))

	def save_model(self, filename):
		self.model.save(filename)

	# This is where we perform actions and signal that the neural net is ready to train
	def coverageMetricCallback(self, metric):
		# Speed has not been set by network before, so initialize with a random speed.
		if (self.env.lidar_pan_speed <= 0):
			action = random.randint(0, self.env.action_shape - 1)
			self.env.performAction(action)
			return

		# Return if we are missing state information
		if (not self.env.linear_velocity) or (not self.env.angular_velocity) \
		    or (self.env.servo_angle <= 0) or (metric.data <= 0):
			return

		# Update the environment state
		self.env.update(metric.data)

		self.iterations += 1
		rospy.loginfo("Training iteration #%s\n", self.iterations)

		self.ready_to_update = True
		return

	def iterateModels(self):
		self.ready_to_update = False

		# Get next action and perform it
		action = self.act(self.env.memory[-1])
		self.env.performAction(action)

		# Can't store results of action on very first iteration
		if len(self.env.memory) < 2:
			self.last_action = action
			return

		# Get reward from previous action
		reward = self.env.getReward()

		# Store the previous state, action taken
		self.remember(self.env.memory[-2], self.last_action, reward, self.env.memory[-1], False)
		self.last_action = action

		# Iterate prediction model and target model
		self.replay()
		if self.iterations%self.tau == 0:
			self.target_train()

def rosShutdownCallback():
	# Save the model to the disk
	global scan_dither_dqn
	scan_dither_dqn.save_model(scan_dither_dqn.model_filename)
	rospy.loginfo("Saved dithering model")

scan_dither_dqn = 0

def main():
	rospy.init_node("adaptive_dither_dqn_node")

	# Create an instance of the adaptive scan dithering neural network
	global scan_dither_dqn 
	scan_dither_dqn = ScanDitherDQN()

	rospy.on_shutdown(rosShutdownCallback)

	while not rospy.is_shutdown():
		if scan_dither_dqn.ready_to_update:
			# For some reason if we call this from within the metrics callback we corrupt the tensors...
			scan_dither_dqn.iterateModels()

if __name__ == "__main__":
	main()
