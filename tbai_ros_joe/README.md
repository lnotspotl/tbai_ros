# tbai_ros_joe package

## Example
```bash
# Start ROS and relevant nodes
roslaunch tbai_ros_joe simple_perceptive.launch gui:=true

# Change controllers
rostopic pub /anymal_d/change_controller std_msgs/String "data: 'STAND'"
rostopic pub /anymal_d/change_controller std_msgs/String "data: 'JOE'"
```


https://github.com/lnotspotl/tbai_ros/assets/82883398/2d4dc8cc-4aa5-4709-91ef-e943dbad87bf

