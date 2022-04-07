from lib.interface.map import MapInterface
from lib.interface.path import PathInterface
from lib.interface.pose import PoseInterface


class BaseMap:
    def __init__(self):
        self.map = MapInterface("/map")
        self.pose = PoseInterface("/slam_out_pose")
        self.path = PathInterface()
        self.frame = np.zeros((600, 600, 3), np.uint8)

    def update(self, end):
        self.frame = np.zeros((600, 600, 3), np.uint8)
        for row in range(0, 600):
            for col in range(0, 600):
                if self.map.bitmap[row][col] == 0:
                    self.frame[row][col] = (166, 166, 166)
                elif self.map.bitmap[row][col] == 1:
                    self.frame[row][col] = (255, 255, 255)
                elif self.map.bitmap[row][col] == 2:
                    self.frame[row][col] = (0, 0, 0)
        self.frame[self.pose.x][self.pose.y] = (255, 0, 0)
        if end is not None:
            pathList = self.path.findPath(self.map.bitmap, (self.pose.x, self.pose.y), end)
            for item in pathList:
                self.frame[item[0]][item[1]] = (0, 0, 255)
        return cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

