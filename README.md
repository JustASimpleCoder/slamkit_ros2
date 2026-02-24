# SLIMKIT ROS package

ROS2 node and test application for slamkit, this is forked from slamtec toolboocx to work with ros2 humble. 

## Dependency

This project need the following dependency:

1. libusb-1.0-0-dev
    ```bash
      sudo apt-get install libusb-1.0-0-dev
    ```
2. imu-tools
   ```bash
    sudo apt-get install ros-\<dist\>-imu-tools
    \<dist> is the distrbution, such as "humble" (tested only on humble as of February 23rd 2026)
   ```
## How to build slamkit ros package

1. Make sure your system have installed ROS
   If you don't familar with ROS, please visit <https://docs.ros.org/en/humble/index.html> firstly, and follow the "Installation" and "Tutorials".
2. Create a workspace for this projectand move to the src/ directory, if you already have one skip to step 3
    ```bash
    mkdir -p ~/slamkit_ws/src
    cd ~/slamkit_ws/src
    ```
3. Clone or copy this project to your ROS2 workspace src folder and init the submodule for the slamware sdk
   ```bash
   git clone <slamkit_ros2 repo>
   cd slamkit_ros2
   git submodule update --init
   ```

3. Go to the slamkit workspace and run colcon_build to build slamkitNode and slamkitNodeClient
   
   ```bash
   cd ~/slamkit_ws
   colcon_build
   source install/setup.bash
   ```

## How to run slamkit ros package

1. Please plug in the SLAMKIT device, and use lsusb cmd to check the device is detected

    ```bash
    lsusb
    ```
    Expected output:
    ```bash
      // other USB devices 
      BUS 00 ....  etc.
      
      // slamkit device
      Bus 003 Device 011: ID fccf:f100 SLAMTEC SLAMWARELC
    ```
    

2. Run the shell script to add udev rule, the script is in the scripts/ folder, change permissions on the file and run as sudo

   ```bash
      cd /slamkit_ws/src/slamkit_ros2/scripts
      chmod +x add_udev.sh
      sudo ./add_udev.sh
   ```


### I. Run slamkit node only

```bash
ros2 launch slamkit_ros2 slamkit_usb.py
```

The slamkitNode will publish 3 topics, the complementary_filter_node will publish: 

```bash
ros2 topic list

output:

/imu/data_raw
/imu/mag
/imu/processed_yaw
/parameter_events
/rosout
```

topic:
- imu/data_raw (sensor_msgs/Imu)
   Message containing raw IMU data, including angular velocities and linear accelerations.
   acc raw data in m/s^2, and gyro in rad/s.

- imu/mag (sensor_msgs/MagneticField)
   Magnetic data in Tesla.

- imu/processed_yaw (geometry_msgs::Vector3Stamped)
   Yaw data which processed in slamkit hardware device, in degree.

### II. Run slamkit node and imu filter node

```bash
ros2 slamkit_ros2 slamkit_usb_imu_filter.py
```

The complementary_filter_node in imu tools will start at the same time, and the topics are:

- imu/data (sensor_msgs/Imu)
   The fused Imu message, containing the orientation.
- imu/rpy/filtered (geometry_msgs/Vector3)
   Debug only: The roll, pitch and yaw angles corresponding to the orientation published on the imu_data topic. (only published when publish_debug_topics == true in the launch file(s))
- imu/steady_state (std_msgs/Bool)
   Debug only: Whether we are in the steady state when doing bias estimation. (only published when publish_debug_topics == true in launch file(s))

### III. View in Rviz

```bash
ros2 slamkit_ros2 slamkit_test.py
```

The slamkit_usb_imu_filter.pywill start the following nodes:

```bash
    complementary_filter_node (imu_complementary_filter/complementary_filter_node)
    rviz (rviz/rviz)
    slamkitNode (slamkit_ros2/slamkitNode)
    slamkitNodeClient (slamkit_ros2/slamkitNodeClient)
```

The rviz will use /imu/data which published from complementary_filter_node and display int window.

If want to view the roll/pich/yaw data, start another teminal and echo the following topic:

- view imu msg (sensor_msgs/Imu) from complementary_filter_node

```bash
j@j ~/works/slamkit_ros_ws $ ros2 topic echo /imu/data
header:
  stamp:
    sec: 1771906176
    nanosec: 292503434
  frame_id: ''
orientation:
  x: -0.0006820655739043123
  y: -0.0008500529982634072
  z: 0.014506068315529183
  w: 0.9998941874910956
orientation_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: -0.0029858178752034253
  y: 0.00029620099814606747
  z: 0.0025040352709017385
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0

---
```

- view roll/pich/yaw in rad

```bash
j@j ~ $ ros2 topic echo /imu/rpy/filtered 
header:
  stamp:
    sec: 1771906234
    nanosec: 690378419
  frame_id: ''
vector:
  x: -9.70745919974312e-05
  y: -0.00023339190810723926
  z: 0.035364466423382306
---
```

- view roll/pich/yaw in degree

```bash
j@j ~ $ ros2 topic echo /imu/angles_degree 
header:
  stamp:
    sec: 1771906296
    nanosec: 1326880
  frame_id: angle_degree
vector:
  x: -0.039784634346500466
  y: -0.15514760924453072
  z: 2.402130786957069
---
```


### III. View slamkit processed yaw (in degree)

```bash
j@j ~ $ rostopic echo /imu/processed_yaw 
header:
  stamp:
    sec: 1771906332
    nanosec: 527674486
  frame_id: imu_processed
vector:
  x: 0.0
  y: 0.0
  z: 45.11469678271026

---
```
