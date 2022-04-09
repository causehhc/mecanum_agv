# mecanum_agv
## INIT
### 1.创建工作空间并初始化
```
git clone https://github.com/causehhc/mecanum_agv.git
cd mecanum_agv
catkin_make
```
### 2.run testNode
```
source ./devel/setup.bash
roslaunch sim_gazebo room_world.launch
```
## START
### 1.start room_world simulation
```
killall gzserver
roslaunch sim_gazebo room_world.launch
```
### 2.check rostopic
`rostopic list`
### 3.start remote car
`conda activate py39 && rosrun sim_gazebo remote_car.py`
### 4.start image view
`rosrun image_view image_view image:=/sim/smallCar/camera/image_raw`
### 5.start hector_mapping SLAM(NEED Third pkg)
`roslaunch rplidar_ros hector_mapping_demo.launch`
### 6.It's time to get some realShit
`roslaunch rplidar_ros hector_mapping_real.launch`
### 7.NAV
`roslaunch navigation test_amcl.launch`

pip install opencv-python==4.3.0.38 -i https://pypi.tuna.tsinghua.edu.cn/simple