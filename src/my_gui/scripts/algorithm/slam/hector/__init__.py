import roslaunch


class Hetor_SLAM:
    def __init__(self):
        self.launch_file = "/home/hhc/Desktop/ros/mecanum_agv/src/my_hector/launch/hector_mapping.launch"

    def start(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, [self.launch_file])
        tracking_launch.start()
