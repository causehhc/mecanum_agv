from algorithm.slam.hector.lib.util.UtilFunctions import *





class ScanMatcher:
    def __init__(self):
        self.dTr = None
        self.H = None

    def estimateTransformationLogLh(self, estimate, gridMapUtil, dataPoints):
        return True

    def matchData(self, beginEstimateWorld, gridMapUtil, dataContainer, covMatrix, maxIterations):
        if dataContainer.getSize() != 0:
            beginEstimateMap = gridMapUtil.getMapCoordsPose(beginEstimateWorld)
            estimate = beginEstimateMap
            self.estimateTransformationLogLh(estimate, gridMapUtil, dataContainer)
            numIter = maxIterations
            for i in range(0, numIter):
                self.estimateTransformationLogLh(estimate, gridMapUtil, dataContainer)
            estimate[2] = normalize_angle(estimate[2])
            # covMatrix = Zero()
            covMatrix = self.H
            return gridMapUtil.getWorldCoordsPose(estimate)
        return beginEstimateWorld
