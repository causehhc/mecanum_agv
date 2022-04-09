class DataPointContainer:
    def __init__(self):
        self.dataPoints = []
        self.origo = None

    def getSize(self):
        return len(self.dataPoints)

    def setFrom(self, other, factor):
        self.origo = other.getOrigo() * factor
        self.dataPoints = other.dataPoints
        size = len(self.dataPoints)
        for i in range(0, size):
            self.dataPoints[i] *= factor

    def clear(self):
        self.dataPoints = []

    def setOrigo(self, origoIn):
        self.origo = origoIn

    def getOrigo(self):
        return self.origo

    def add(self, dataPoint):
        self.dataPoints.append(dataPoint)
