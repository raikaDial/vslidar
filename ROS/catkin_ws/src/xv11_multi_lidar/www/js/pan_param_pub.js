class PanParamReconfigure {
	constructor(ros_master_ip) {
		// Connect to the ROS Master
		this.ros = new ROSLIB.Ros({
			url : 'ws://' + ros_master_ip + ':9090'
		});

		// Setup node callbacks
		this.ros.on('connection', function() {
			console.log('Connected to websocket Server');
		});

		this.ros.on('error', function(error) {
			console.log('Error connecting to websocket server', error);
		});

		this.ros.on('close', function() {
			console.log('Connection to websocket server closed.');
		});

		// Create Publishers and Subscribers
		this.pan_param_pub = new ROSLIB.Topic({
			ros : this.ros,
			name : 'pan_params',
			messageType : 'std_msgs/Int16MultiArray'
		});

	}

	publishPanningParams() {
		var lidar_id = parseInt(document.getElementById('lidarIDSelectID').value);
		var rotation_speed = parseInt(document.getElementById('rotationSpeedInputId').value);
		var cw_limit = parseInt(document.getElementById('cwLimitInputId').value);
		var ccw_limit = parseInt(document.getElementById('ccwLimitInputId').value);

		var param_msg = new ROSLIB.Message ({
			data : [lidar_id, cw_limit, ccw_limit, rotation_speed],
			layout : {
				dim : [{
					size : 4,
					stride : 1,
					label : "pan_params"
				}]
			}
		});

		console.log("Publishing Params.")
		this.pan_param_pub.publish(param_msg);			
	}
}

var pan_param_reconfigure;

function initialize_ros(ip) {
	pan_param_reconfigure = new PanParamReconfigure(ip);
}

// This should actually be called by some sort of IP entry box, 
//     unless we set a static IP for the robot.
initialize_ros("127.0.0.1") //"10.10.10.101")

