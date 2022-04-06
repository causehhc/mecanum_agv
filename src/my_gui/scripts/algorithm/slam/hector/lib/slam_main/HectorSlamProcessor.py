from algorithm.slam.hector.lib.util.UtilFunctions import *
from algorithm.slam.hector.lib.slam_main.MapRepMultiMap import MapRepMultiMap


class HectorSlamProcessor:
    def __init__(self):
        self.lastScanMatchPose = None
        self.lastMapUpdatePose = None
        self.lastScanMatchCov = None
        self.paramMinDistanceDiffForMapUpdate = None
        self.paramMinAngleDiffForMapUpdate = None
        self.mapRep = MapRepMultiMap()

    def getScaleToMap(self):
        return self.mapRep.getScaleToMap()

    def getLastScanMatchPose(self):
        # TODO
        return True

    def update(self, dataContainer, poseHintWorld):
        newPoseEstimateWorld = self.mapRep.matchData(poseHintWorld, dataContainer, self.lastScanMatchCov)
        self.lastScanMatchPose = newPoseEstimateWorld
        if poseDifferenceLargerThan(newPoseEstimateWorld, self.lastMapUpdatePose, self.paramMinDistanceDiffForMapUpdate, self.paramMinAngleDiffForMapUpdate):
            self.mapRep.updateByScan(dataContainer, newPoseEstimateWorld)
            self.mapRep.onMapUpdated()
            self.lastMapUpdatePose = newPoseEstimateWorld
