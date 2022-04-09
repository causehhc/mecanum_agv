import math

from algorithm.slam.hector.lib.slam_main.HectorSlamProcessor import HectorSlamProcessor
from algorithm.slam.hector.lib.scan.DataPointContainer import DataPointContainer


class HectorMappingRos:
    def __init__(self):
        self.laserScanContainer = None
        self.slamProcessor = HectorSlamProcessor()
        self.dataContainer = DataPointContainer()

    def rosLaserScanToDataContainer(self, scan, dataContainer, scaleToMap):
        size = len(scan.ranges)
        angle = scan.angle_min
        dataContainer.clear()
        dataContainer.setOrigo([0, 0])
        maxRangeForContainer = scan.range_max - 0.1
        for i in range(0, size):
            dist = scan.ranges[i]
            if scan.range_min < dist < maxRangeForContainer:
                dist *= scaleToMap
                dataContainer.add([math.cos(angle) * dist, math.sin(angle) * dist])
            angle += scan.angle_increment

    def scanCallback(self, scan):
        self.rosLaserScanToDataContainer(scan, self.laserScanContainer, self.slamProcessor.getScaleToMap())
        self.slamProcessor.update(self.laserScanContainer, self.slamProcessor.getLastScanMatchPose())
        # TODO
        # poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(), slamProcessor->getLastScanMatchCovariance(), scan.header.stamp, p_map_frame_);
        # poseUpdatePublisher_.publish(poseInfoContainer_.getPoseWithCovarianceStamped());
        # posePublisher_.publish(poseInfoContainer_.getPoseStamped());
