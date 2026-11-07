first source the ros 2 file with 
source /opt/ros/jazzy/setup.bash

you may have to build the folder first with colcon
colcon build --symlink-install

then move into the folder and source the workspace within the nav2_test
cd *the_folder*
source install/setup.bash

then you can run the simulation with 
ros2 launch nav2_test robot_sim.launch.py

the robot should be in the middle of the map, under the second level, use the move tool to move it out.

open another terminal to run the controller to control the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

to record, use the command 
ros2 bag record --use-sim-time \
  -o /home/jun/Desktop/robot_run \
  /camera/image /odom
  
to convert the recording to mp4, use the command
ros2 bag to_video /home/jun/Desktop/robot_run \
  --topic /camera/image \
  --output /home/jun/Desktop/robot_run.mp4 \
  --fps 24 \
  --codec mp4v
