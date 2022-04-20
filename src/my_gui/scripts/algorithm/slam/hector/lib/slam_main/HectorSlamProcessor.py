import numpy as np
from algorithm.slam.hector.lib.util.UtilFunctions import *
from algorithm.slam.hector.lib.slam_main.MapRepMultiMap import MapRepMultiMap


class HectorSlamProcessor:
    def __init__(self, mapResolution, mapSizeX, mapSizeY, startCoords, multi_res_size):
        self._lastMapUpdatePose = None
        self._lastScanMatchPose = None
        self._lastScanMatchCov = None
        self._paramMinDistanceDiffForMapUpdate = None
        self._paramMinAngleDiffForMapUpdate = None

        self.mapRep = MapRepMultiMap(mapResolution, mapSizeX, mapSizeY, multi_res_size, startCoords)
        self.reset()
        self.setMapUpdateMinDistDiff(0.4)
        self.setMapUpdateMinAngleDiff(0.13)

    def update(self, dataContainer, poseHintWorld):
        newPoseEstimateWorld = self.mapRep.matchData(
            poseHintWorld,
            dataContainer,
            self._lastScanMatchCov
        )
        self._lastScanMatchPose = newPoseEstimateWorld
        if poseDifferenceLargerThan(
                newPoseEstimateWorld,
                self._lastMapUpdatePose,
                self._paramMinDistanceDiffForMapUpdate,
                self._paramMinAngleDiffForMapUpdate
        ):
            self.mapRep.updateByScan(dataContainer, newPoseEstimateWorld)
            self.mapRep.onMapUpdated()
            self._lastMapUpdatePose = newPoseEstimateWorld

    def reset(self):
        self._lastMapUpdatePose = np.ones((3, 1), dtype=np.float)
        self._lastScanMatchPose = np.zeros((3, 1), dtype=np.float)
        self.mapRep.reset()

    def getLastScanMatchPose(self):
        return self._lastScanMatchPose

    def getLastScanMatchCovariance(self):
        return self._lastScanMatchCov

    def getScaleToMap(self):
        return self.mapRep.getScaleToMap()

    def getMapLevels(self):
        return self.mapRep.getMapLevels()

    def getGridMap(self, mapLevel=0):
        return self.mapRep.getGridMap(mapLevel)

    def setUpdateFactorFree(self, free_factor):
        self.mapRep.setUpdateFactorFree(free_factor)

    def setUpdateFactorOccupied(self, occupied_factor):
        self.mapRep.setUpdateFactorOccupied(occupied_factor)

    def setMapUpdateMinDistDiff(self, minDist):
        self._paramMinDistanceDiffForMapUpdate = minDist

    def setMapUpdateMinAngleDiff(self, angleChange):
        self._paramMinAngleDiffForMapUpdate = angleChange
