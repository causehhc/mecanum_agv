import roslaunch


class Hetor_SLAM:
    def __init__(self):
        self.launch_file = "/home/hhc/Desktop/ros/mecanum_agv/src/my_hector/launch/hector_mapping.launch"
        self.tracking_launch = None
        self.status = False

    def config(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, [self.launch_file])

    def start(self):
        self.tracking_launch.start()
        self.status = True

    def stop(self):
        self.tracking_launch.shutdown()
        self.status = False
