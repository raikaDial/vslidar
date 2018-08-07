## Build/Install
* Make sure you have ROS installed. The system was designed and tested with ROS Indigo running on Ubuntu 14.04, but it may work in newer versions.
* Clone this repository
* cd into the catkin_ws directory
* Clone the code for the submodules  
`git submodule update --init --remote --recursive`
* Install ROS dependencies  
`rosdep update`  
`rosdep install --from-paths src --ignore-src -r -y`
* Install ninja-build  
`sudo apt-get update`  
`sudo apt-get install ninja-build`
* Install protobuff 3  
`src/cartographer/scripts/install_proto3.sh`
* Build the system  
`catkin_make_isolated --install --use-ninja`

## Environment Setup
Quality of life things to make spinup easier:  

* Add the following line to your .bashrc (obviously, replacing <path\_to\_catkin\_ws> with the actual path...)
`source <path_to_catkin_ws>/install_isolated/setup.bash` 
* Also adding to the bashrc:
    * If primary network interface is wireless  
`` export ROS_HOSTNAME=`ifconfig wlan0 2>/dev/null|awk '/inet addr:/ {print $2}'|sed 's/addr://'` ``
    * If primary network interface is wired  
`` export ROS_HOSTNAME=`ifconfig eth0 2>/dev/null|awk '/inet addr:/ {print $2}'|sed 's/addr://'` ``
* On the control laptop, also add to the bashrc:  
`` export ROS_MASTER_URI=http://`ifconfig wlan0 2>/dev/null|awk '/inet addr:/ {print $2}'|sed 's/addr://'`:11311 ``

## Running
* To start the VS-LIDAR system, run on the control laptop:
`roslaunch vslidar_autonomy vslidar_autonomy.launch`
    * Of course, make sure the system is plugged in first.


## Other Important Notes
* Some of the xv11\_multi\_lidar nodes will segfault if you compile them with c++11 support. This seems to be a bug with the point cloud library.
* If you try to build the system without installing it (e.g. `catkin_make_isolated --use-ninja`), building cartographer may crash with the error *Unrecognized syntax identifier "proto3".  This parser only recognizes "proto2".*
* If you try to download and use a newer version of Cartographer, beware that their LUA configuration dictionary has likely been updated, and the cartographer node will crash unless you add the new keys to the config file in *<path\_to\_catkin\_ws>/src/vslidar\_autonomy/config/vslidar\_cartographer\_config.lua*. Usually, there are some example config files in the cartographer\_ros repository on GitHub that are kept up-to-date with all the required keys included (but you will have to figure out what values to use for those keys on your own). You will also need to remake the catkin after updating the config file.
