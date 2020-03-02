# How to run
## 1. Gazebo Test
add following plugin to feed ground truth pose data
```
    <plugin name="gazebo_ros_p3d" filename="libgazebo_ros_p3d.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>100.0</updateRate>
      <bodyName>base_link</bodyName>
      <topicName>ground_truth_pose</topicName>
      <gaussianNoise>0.0</gaussianNoise>
      <frameName>world</frameName>
      <xyzOffsets>0 0 0</xyzOffsets>	
      <rpyOffsets>0 0 0</rpyOffsets>
    </plugin>
```

copy & paste premade "warehouse_boxes.world" in /world folder to following directory
```
<PX4 Firmware dir>/Tools/sitl_gazebo/worlds
```

source SITL environment (may not be needed)
```
cd <PX4 Firmware dir>
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
```

run Gazebo simulation & Controller
```
roslaunch Height_Estimation IRIS_CTRL.launch 
```

run algorithm
```
rosrun Height_Estimation main_VIO.py
```

## 2. Hardware Test
in "<realsense-ros dir>/realsense2_camera/launch/includes/nodelet.launch.xml" file, add following statements at the end of the node
```
    <rosparam>
        /camera/tracking_module/enable_mapping: false
        /camera/tracking_module/enable_pose_jumping: false
        /camera/tracking_module/enable_relocalization: false
    </rosparam>

    <remap from="pose/sample" to="/vio_pose_in" if="$(arg px4)"/>
```
run algorithm
```
rosrun Height_Estimation main_VIO.py
```

# user can choose mode change policy
## automatic mode change between rangefinder and VIO
```
rosservice call hgt_meas_source_selection_policy 0
```

## only rangefinder 
```
rosservice call hgt_meas_source_selection_policy 1
```

## only VIO 
```
rosservice call hgt_meas_source_selection_policy 2
```
