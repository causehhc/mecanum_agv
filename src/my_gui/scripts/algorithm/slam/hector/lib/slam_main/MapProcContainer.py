from algorithm.slam.hector.lib.matcher.ScanMatcher import ScanMatcher
class MapProcContainer:
    def __init__(self):
        self.scanMatcher = ScanMatcher()
        self.gridMapUtil = None

    def resetCachedData(self):
        # TODO
        pass

    def matchData(self, beginEstimateWorld, dataContainer, covMatrix, maxIterations):
        return self.scanMatcher.matchData(beginEstimateWorld, self.gridMapUtil, dataContainer, covMatrix, maxIterations)

    def updateByScan(self, dataContainer, robotPoseWorld):
        # TODO
        pass

    def getScaleToMap(self):
        # TODO
        return self.gridMap.getScaleToMap()
