import rospy
from sensor_msgs.msg import LaserScan


class OccGridMapUtilConfig:
    def __init__(self):
        pass

    def getMapCoordsPose(self, worldPose):
        pass

    def getCompleteHessianDerivs(self, pose, dataPoints, H, dTr):
        pass


def estimateTransformationLogLh(estimate, gridMapUtil, dataPoints):
    """
    :param estimate: t-1时刻机器人在地图坐标系下的位姿
    :param gridMapUtil: 栅格地图
    :param dataPoints: 激光数据
    :return:
    """
    gridMapUtil.getCompleteHessianDerivs(estimate, dataPoints, H, dTr)
    if H[0][0] != 0 and H[1][1] != 0:
        searchDir = H.inverse() * dTr
        if searchDir[2] > 0.2:
            searchDir[2] = 0.2
        elif searchDir[2] < -0.2:
            searchDir[2] = -0.2
        updateEstimatedPose(estimate, searchDir)
        return True
    return False


class ScanMatcher:
    def __init__(self):
        pass

    def matchDate(self, beginEstimateWorld, gridMapUtil, dataContainer, covMatrix, maxIterations):
        """
        重点中的核心，飞机中的战斗机
        :param beginEstimateWorld: t-1时刻机器人在世界坐标系下的位姿
        :param gridMapUtil: 栅格地图
        :param dataContainer: 激光数据
        :param covMatrix: 当前时刻hassian矩阵
        :param maxIterations: 最大迭代次数
        :return: t时刻机器人在世界坐标系下的位姿
        """
        if len(dataContainer) != 0:
            beginEstimateMap = gridMapUtil.getMapCoordsPose(beginEstimateWorld)
            estimate = beginEstimateMap
            estimateTransformationLogLh(estimate, gridMapUtil, dataContainer)
            for i in range(0, maxIterations):
                estimateTransformationLogLh(estimate, gridMapUtil, dataContainer)
            return gridMapUtil.getWorldCoordsPose(estimate)
        return beginEstimateWorld


class MapProcContainer:
    def __init__(self):
        self.scanMatcher = ScanMatcher()
        self.gridMapUtil = 1

    def matchData(self, beginEstimateWorld, dataContainer, covMatrix, maxIterations):
        return self.scanMatcher.matchDate(beginEstimateWorld, self.gridMapUtil, dataContainer, covMatrix, maxIterations)


class MapRepresentationInterface:
    def __init__(self):
        self.mapContainer = []
        self.dataContainers = []

    def matchData(self, beginEstimateWorld, dataContainer, covMatrix):
        """
        默认三层地图，分辨率0.025m 0.05m 0.1m 从0.1m层开始处理
        论文中说，使用分层地图计算是为了避免陷入局部最小值，且其地图的分层，并不是通过高分辨率的地图降采样得到，
        而是使用不同的地图存储器，每种分辨率的地图单独极端，从低分辨率的地图开始进行扫描匹配，然后将方程的解迭代进入高精度的地图再解算，多次迭代后，得到当前时刻，机器人的位姿（更精确）
        另一个优点是：使用多分辨率的地图，可是导航，路径规划更高效。
        """
        size = len(self.mapContainer)
        tmp = beginEstimateWorld
        for index in range(size - 1, 0):
            if index == 0:
                tmp = self.mapContainer[index].matchData(tmp, dataContainer, covMatrix, 5)
            else:
                self.dataContainers[index - 1].setFrom(dataContainer, 0)
                tmp = self.mapContainer[index].matchData(tmp, self.dataContainers[index - 1], covMatrix, 3)
        return tmp

    def updateByScan(self, dataContainer, robotPoseWorld):
        pass

    def onMapUpdated(self):
        size = len(self.mapContainer)
        for i in range(0, size):
            self.mapContainer[i].resetCachedData()


class SlamProcess:
    def __init__(self):
        self.M_PI = 3.14159265358979323846
        self.laseScanMatchCov = None
        self.laseScanMatchPose = None
        self.lastMapUpdatePose = None
        self.paramMinDistanceDiffForMapUpdate = 0.4
        self.paramMinAngleDiffForMapUpdate = 0.13

    def poseDifferenceLargerThan(self, pose1, pose2, distanceDiffThresh, angleDiffThresh):
        if (pose1[0][2] - pose2[0][2]) > distanceDiffThresh:
            return True
        angleDiff = pose1[1] - pose2[1]
        if angleDiff > self.M_PI:
            angleDiff -= self.M_PI * 2.0
        elif angleDiff < -self.M_PI:
            angleDiff += self.M_PI * 2.0
        if abs(angleDiff) > angleDiffThresh:
            return True
        return False

    def update(self, dataContainer, possHintWorld, map_without_matching=False):
        if map_without_matching is not True:
            newPoseEstimateWorld = mapRep.matchData(possHintWorld, dataContainer, self.laseScanMatchCov)
        else:
            newPoseEstimateWorld = possHintWorld
        self.laseScanMatchPose = newPoseEstimateWorld
        if self.poseDifferenceLargerThan(newPoseEstimateWorld, self.lastMapUpdatePose,
                                         self.paramMinDistanceDiffForMapUpdate,
                                         self.paramMinAngleDiffForMapUpdate) or map_without_matching:
            mapRep.updateByScan(dataContainer, newPoseEstimateWorld)
            mapRep.onMapUpdated()
            self.lastMapUpdatePose = newPoseEstimateWorld


def ScanCallback(scan):
    slamProcessor.update(1, 1)


def main():
    rospy.init_node('hector_map', anonymous=True)
    rospy.Subscriber("/sim/smallCar/laser/scan", LaserScan, ScanCallback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    slamProcessor = SlamProcess()
    mapRep = MapRepresentationInterface()
    main()
