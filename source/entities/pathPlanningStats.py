class PathPlanningStats:
    def __init__(self):
        self.initTimesamp = 0
        self.finalTimestamp = 0
        self.safeScale = 0
        self.mapScale = 0
        self.map = []
        self.iterationsCounter = 0
        self.visitedNodes = 0
        self.pathSize = 0

