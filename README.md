# LIDARSTREAM

Project for Streaming of LIDAR Data to any client.

### Required OS System ###

* This project can be run ubuntu 16.04(kinetic) System installed ROS(Robotic Operating System)

### Command List for Installing ROS Command on RAW Ubuntu system ###

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### Deploy ros_datafeed module.

- `ros_datafeed` is the customized service from ros modules, and is used to get the point cloud from ros core and sent it to Look3D by TCP socket.
- Extract `LiSTREAM.rar`(arbitrary directory).

### Run LiDAR Stream Server ###

Every command should be run on each individual terminal as these are daemons.

* `roscore`
* `rar_path/LiSTREAM/ros_datafeed/ros_datafeed_cpp/devel/lib/rosserial_server/.ros_datafeed`
* `rosbag play -l [bagfile]` in bagfile path
