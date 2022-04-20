class MapProcContainer:
    def __init__(self, gridMapIn, gridMapUtilIn, scanMatcherIn):
        self._gridMap = gridMapIn
        self._gridMapUtil = gridMapUtilIn
        self._scanMatcher = scanMatcherIn

    def cleanup(self):
        self._gridMap = None
        self._gridMapUtil = None
        self._scanMatcher = None

    def reset(self):
        self._gridMap.reset()
        self._gridMapUtil.resetCachedData()

    def resetCachedData(self):
        self._gridMapUtil.resetCachedData()

    def getScaleToMap(self):
        return self._gridMap.getScaleToMap()

    def getGridMap(self):
        return self._gridMap

    def matchData(self, beginEstimateWorld, dataContainer, covMatrix, maxIterations):
        return self._scanMatcher.matchData(beginEstimateWorld, self._gridMapUtil, dataContainer, covMatrix,
                                           maxIterations)

    def updateByScan(self, dataContainer, robotPoseWorld):
        self._gridMap.updateByScan(dataContainer, robotPoseWorld)
