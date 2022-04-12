import roslaunch


class Hetor_SLAM:
    def __init__(self):
        self.launch_file = "/home/hhc/Desktop/ros/bishe_ws/src/my_gui/launch/hector_mapping_real.launch"

    def start(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, [self.launch_file])
        tracking_launch.start()
