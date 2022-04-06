from algorithm.slam.hector.lib.slam_main.MapProcContainer import MapProcContainer
from algorithm.slam.hector.lib.scan.DataPointContainer import DataPointContainer


class MapRepMultiMap:
    def __init__(self):
        self.mapContainer = [MapProcContainer()]
        self.dataContainers = [DataPointContainer()]

    def onMapUpdated(self):
        size = len(self.mapContainer)
        for i in range(0, size):
            self.mapContainer[i].resetCachedData()

    def matchData(self, beginEstimateWorld, dataContainer, covMatrix):
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
        size = len(self.mapContainer)
        for i in range(0, size):
            if i == 0:
                self.mapContainer[i].updateByScan(dataContainer, robotPoseWorld)
            else:
                self.mapContainer[i].updateByScan(self.dataContainers[i - 1], robotPoseWorld)
        return True

    def getScaleToMap(self):
        return self.mapContainer[0].getScaleToMap()
