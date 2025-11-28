# tbai_ros_dtc package

## Example
```bash
# Start ROS and relevant nodes
roslaunch tbai_ros_dtc simple_blind.launch gui:=true # blind version
roslaunch tbai_ros_dtc simple_perceptive.launch gui:=true # blind version

# Change controllers
rostopic pub /anymal_d/change_controller std_msgs/String "data: 'STAND'"
rostopic pub /anymal_d/change_controller std_msgs/String "data: 'DTC'"
```


https://github.com/lnotspotl/tbai_ros/assets/82883398/0ecef030-1581-4400-9e6a-9df2fadb3d3a

