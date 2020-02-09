roslaunch delayed_odometry turtlebot_gazebo.launch
rosrun delayed_odometry odometry_delayer _delay:=0.05 _rate:=100

rosrun turtlebot_trajectory_controller trajectory_controller_node

rosrun turtlebot_trajectory_testing send_test_trajectory _fw_vel:=.35 _period:=4 _mag:=.5
rostopic pub /mobile_base/events/button kobuki_msgs/ButtonEvent "button: 0
state: 0" 
