# start room_world simulation
- killall gzserver
- roslaunch sim_gazebo room_world.launch
# check rostopic
- rostopic list
# start remote car
- conda activate py39 && rosrun sim_gazebo remote_car.py
# start image view
- rosrun image_view image_view image:=/sim/smallCar/camera/image_raw
# start hector_mapping SLAM(NEED Third pkg)
- git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam
- roslaunch rplidar_ros hector_mapping_demo.launch