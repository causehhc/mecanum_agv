class DataPointContainer:
    def __init__(self, size=1000):
        self._dataPoints = []
        self._origo = None

    def setFrom(self, other, factor):
        self._origo = other.getOrigo() * factor
        self._dataPoints = other._dataPoints
        size = len(self._dataPoints)
        for i in range(0, size):
            self._dataPoints[i] *= factor

    def add(self, dataPoint):
        self._dataPoints.append(dataPoint)

    def clear(self):
        self._dataPoints = []

    def getSize(self):
        return len(self._dataPoints)

    def getVecEntry(self, index):
        return self._dataPoints[index]

    def getOrigo(self):
        return self._origo

    def setOrigo(self, origoIn):
        self._origo = origoIn
