import numpy as np

from algorithm.slam.hector.lib.slam_main.MapProcContainer import MapProcContainer
from algorithm.slam.hector.lib.scan.DataPointContainer import DataPointContainer


class MapRepMultiMap:
    def __init__(self, mapResolution, mapSizeX, mapSizeY, numDepth, startCoords):
        self._mapContainer = []
        self._dataContainers = []

        resolution = np.array([[mapSizeX], [mapSizeY]], dtype=int)
        totalMapSizeX = mapResolution * float(mapSizeX)
        mid_offset_x = totalMapSizeX * startCoords.x()
        totalMapSizeY = mapResolution * float(mapSizeY)
        mid_offset_y = totalMapSizeY * startCoords.y()
        for i in range(0, numDepth):
            gridMap = GridMap(mapResolution, resolution, np.array([[mid_offset_x], [mid_offset_y]], dtype=int))
            gridMapUtil = OccGridMapUtilConfig(gridMap)
            scanMatcher = ScanMatcher()
            self._mapContainer.append(MapProcContainer(gridMap, gridMapUtil, scanMatcher))
            resolution /= 2
            mapResolution *= 2.0

    def reset(self):
        size = len(self._mapContainer)
        for i in range(0, size):
            self._mapContainer[i].cleanup()

    def getScaleToMap(self):
        return self._mapContainer[0].getScaleToMap()

    def getMapLevels(self):
        return len(self._mapContainer)

    def getGridMap(self, mapLevel):
        return self._mapContainer[mapLevel].getGridMap()

    def onMapUpdated(self):
        size = len(self._mapContainer)
        for i in range(0, size):
            self._mapContainer[i].resetCachedData()

    def matchData(self, beginEstimateWorld, dataContainer, covMatrix):
        size = len(self._mapContainer)
        tmp = beginEstimateWorld
        for index in range(size - 1, 0):
            if index == 0:
                tmp = self._mapContainer[index].matchData(tmp, dataContainer, covMatrix, 5)
            else:
                self._dataContainers[index - 1].setFrom(dataContainer, 0)
                tmp = self._mapContainer[index].matchData(tmp, self._dataContainers[index - 1], covMatrix, 3)
        return tmp

    def updateByScan(self, dataContainer, robotPoseWorld):
        size = len(self._mapContainer)
        for i in range(0, size):
            if i == 0:
                self._mapContainer[i].updateByScan(dataContainer, robotPoseWorld)
            else:
                self._mapContainer[i].updateByScan(self._dataContainers[i - 1], robotPoseWorld)
        return True

    def setUpdateFactorFree(self, free_factor):
        size = len(self._mapContainer)
        for i in range(0, size):
            map = self._mapContainer[i].getGridMap()
            map.setUpdateFreeFactor(free_factor)

    def setUpdateFactorOccupied(self, occupied_factor):
        size = len(self._mapContainer)
        for i in range(0, size):
            map = self._mapContainer[i].getGridMap()
            map.setUpdateOccupiedFactor(occupied_factor)






