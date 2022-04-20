import numpy as np


class GridMapBase:
    def __init__(self, mapResolution, size, offset):
        newMapDimensions = np.array([[size[0]], [size[1]]], dtype=int)
        self.setMapGridSize(newMapDimensions)
        sizeX = size[0]
        setMapTransformation(offset, mapResolution);FUCKFUCK
        self.scaleToMap = None

    def getScaleToMap(self):
        return self.scaleToMap
