from __future__ import annotations
import math
# from typing import overload

class Constant:
    INITIAL_STATION_PLACEMENT_DISTANCE =  20# meters
    MAX_UAV_DISTANCE = 200 # meters

class Point:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

    def getDistanceFrom(self, pt: Point):
        return ((self.x - pt.x) ** 2 + (self.y - pt.y) ** 2) ** 0.5
    
    def getGradient(self, pt: Point):
        return (pt.y - self.y) / (pt.x - self.x) if pt.x != self.x else 'inf'
    
    def getAngleOfGradient(self, pt: Point):
        return math.atan2((pt.y - self.y) , (pt.x - self.x))
    
    def print(self):
        print(f"({self.x}, {self.y})")

class CornerPoint(Point):
    def __init__(self, point, polygon: Polygon) -> None:
        super().__init__(point[0], point[1])
        self.covered_by : list[Station] = []
        self.polygons : list[Polygon] = [polygon]

    def isCovered(self):
        return len(self.covered_by) > 0
    
    def isCoveredBy(self, station):
        return self.covered_by.count(station) > 0
    
    def addCoverageBy(self, station):
        if not self.isCoveredBy(station):
            self.covered_by.append(station)

    def removeCoverageBy(self, station):
        if self.covered_by.count(station) > 0:
            self.covered_by.remove(station)

    def isPartOf(self, polygon: Polygon):
        return self.polygons.count(polygon) > 0
    
    def isPartOf(self, polygon_id):
        for polygon in self.polygons:
            if polygon.id == polygon_id:
                return True
        return False


class Polygon:
    def __init__(self, id, shape) -> None:
        self.id = id
        self.corner_points : list[CornerPoint] = [CornerPoint(shape[i], self) for i in range(len(shape))]
        self.shape_string = shape
        self.fully_covered_by : list[Station] = []

    def isFullyCovered(self):
        return len(self.fully_covered_by) > 0
    
    def isFullyCoveredBy(self, station):
        return self.fully_covered_by.count(station) > 0
    
    def removeCoverageByStation(self, station: Station):
        self.fully_covered_by.remove(station)
    
    def getCoveringStationIds(self):
        return [ station.id for station in self.fully_covered_by]

class Road:
    def __init__(self, id, width, centerLine) -> None:
        self.id = id
        self.width = width
        self.centerLine = [Point(pt[0], pt[1]) for pt in centerLine]

    def getCenterLineStringList(self):
        return ["({}, {})".format(pt.x, pt.y) for pt in self.centerLine]

    def getAllPossibleStationLocation(self):
        locations = []
        placement_distance = Constant.INITIAL_STATION_PLACEMENT_DISTANCE
        Logger.Write("Road : {}, Direction {}{}, Gradient : {}, Angle : {}".format(self.id, '\u2192' if self.centerLine[0].x < self.centerLine[1].x else '\u2190' if self.centerLine[0].x > self.centerLine[1].x else '\u2194', '\u2191' if self.centerLine[0].y < self.centerLine[1].y else '\u2193' if self.centerLine[0].y > self.centerLine[1].y else '\u2195', self.centerLine[0].getGradient(self.centerLine[1]), self.centerLine[0].getAngleOfGradient(self.centerLine[1])))
        for i in range(len(self.centerLine) - 1):
            distance = self.centerLine[i].getDistanceFrom(self.centerLine[i+1])
            if distance < placement_distance:
                placement_distance -= distance
                continue
            else:
                angle = self.centerLine[i].getAngleOfGradient(self.centerLine[i+1])
                x = self.centerLine[i].x
                y = self.centerLine[i].y
                while placement_distance < distance:
                    x = (placement_distance * self.centerLine[i+1].x + (distance - placement_distance) * x) / distance
                    y = (placement_distance * self.centerLine[i+1].y + (distance - placement_distance) * y) / distance
                    # To do : 
                    xPlacement = x - self.width * math.sin(angle)
                    yPlacement = y - self.width * math.cos(angle)
                    locations.append(Point(xPlacement, yPlacement))
                    distance = self.centerLine[i+1].getDistanceFrom(Point(x, y))
                    placement_distance = Constant.INITIAL_STATION_PLACEMENT_DISTANCE
                placement_distance -= distance
        return locations
    
class Station:
    count = 0
    def __init__(self, location: Point, road: Road) -> None:
        Station.count += 1
        self.id = Station.count
        self.location = location
        self.at_road = road
        self.points_in_range : list[CornerPoint] = []
        self.coveredPolygon : list[Polygon] = []

    def printStation(self):
        print("id : {}, location = ({}, {}), road = {}".format(self.id, self.location.x, self.location.y, self.at_road.id))

    def getXaxisRange(self):
        return self.location.x - Constant.MAX_UAV_DISTANCE, self.location.x + Constant.MAX_UAV_DISTANCE

    def getYaxisRange(self):
        return self.location.y - Constant.MAX_UAV_DISTANCE, self.location.y + Constant.MAX_UAV_DISTANCE
    
    def findAllPointsInRange(self, allPointsSorted : list[CornerPoint]):
        xmin, xmax = self.getXaxisRange()
        # ymin, ymax = self.getYaxisRange()
        index = binarySearchStartingPoint(allPointsSorted, xmin)
        while index < len(allPointsSorted) and allPointsSorted[index].x <= xmax:
            if self.location.getDistanceFrom(allPointsSorted[index]) <= Constant.MAX_UAV_DISTANCE:
                self.points_in_range.append(allPointsSorted[index])
            index += 1
        return self.points_in_range
    
    def determineCoveredPolygon(self):
        checked = [False for i in range(len(self.points_in_range))]
        for i in range(len(self.points_in_range)):
            if checked[i]:
                continue
            for polygon in self.points_in_range[i].polygons:
                isCovered = True
                for point in polygon.corner_points:
                    if self.points_in_range.count(point) > 0:
                        checked[self.points_in_range.index(point)] = True
                    else:
                        isCovered = False
                if isCovered:
                    self.coveredPolygon.append(polygon)
                    polygon.fully_covered_by.append(self)
            checked[i] = True
    
    def prepareForRemoval(self):
        for point in self.points_in_range:
            point.removeCoverageBy(self)
        for polygon in self.coveredPolygon:
            polygon.removeCoverageByStation(self)
        Station.count -= 1

class Logger:
    file = None

    @staticmethod
    def Initialize(filename):
        Logger.file = open(filename, "w")

    @staticmethod
    def Write(msg):
        Logger.file.write(msg+"\n")

    @staticmethod
    def Close():
        Logger.file.close()

def binarySearchStartingPoint(allSortedPoints: list[CornerPoint], xmin):
    start = 0
    end = len(allSortedPoints) - 1
    while start <= end:
        mid = (start + end) // 2
        if allSortedPoints[mid].x == xmin:
            return mid
        elif allSortedPoints[mid].x < xmin:
            start = mid + 1
        else:
            end = mid - 1
    if start >= len(allSortedPoints) or allSortedPoints[start].x > xmin:
        return start
    else:
        return start+1
    
