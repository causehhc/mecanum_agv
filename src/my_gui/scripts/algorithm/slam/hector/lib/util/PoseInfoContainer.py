import math

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header
from tf import Transformer


class PoseInfoContainer:
    def __init__(self):
        self._stampedPose_ = PoseStamped()
        self._covPose_ = PoseWithCovarianceStamped()
        # TODO
        # self._poseTransform_ = Transformer()

    def update(self, slamPose, slamCov, stamp, frame_id):
        header = self._stampedPose_.header
        header.stamp = stamp
        header.frame_id = frame_id

        pose = self._stampedPose_.pose
        pose.position.x = slamPose.x()
        pose.position.y = slamPose.y()

        pose.orientation.w = math.cos(slamPose.z() * 0.5)
        pose.orientation.z = math.sin(slamPose.z() * 0.5)

        self._covPose_.header = header
        self._covPose_.pose.pose = pose

        cov = self._covPose_.pose.covariance.copy()
        cov[0] = float(slamCov(0, 0))
        cov[7] = float(slamCov(1, 1))
        cov[35] = float(slamCov(2, 2))

        xyC = float(slamCov(0, 1))
        cov[1] = xyC
        cov[6] = xyC

        xaC = float(slamCov(0, 2))
        cov[5] = xaC
        cov[30] = xaC

        yaC = float(slamCov(1, 2))
        cov[11] = yaC
        cov[31] = yaC

        # TODO
        # self._poseTransform_ = self._poseTransform_
