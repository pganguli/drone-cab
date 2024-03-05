import traci
import sumolib
import numpy as np
import pulp as pl
import sys
from Utility import *
from typing import List, Tuple
import random


def getTypewisePolygonCount() -> dict:
    polyTypeCnt = {}
    polyIds = traci.polygon.getIDList()
    cnt=0
    for id in polyIds:
        polyType = traci.polygon.getType(id)
        if polyType not in polyTypeCnt.keys():
            polyTypeCnt[polyType] = 1
        else:
            polyTypeCnt[polyType] += 1
        cnt+=1
        doPrint("\r scanning polygons {}/{}...".format(cnt, len(polyIds)),end="", inLogFile=False)
    doPrint("Total polygons scanned : {}".format(cnt))    
    # print(polyTypeCnt)
    return polyTypeCnt

def showAvailablePolygonTypes(polygon_types):
    for i in range(0, len(polygon_types.keys())):
        print("{}. {}({})".format(i+1, list(polygon_types.keys())[i], polygon_types[list(polygon_types.keys())[i]]))

def isSelectionValid(indices, ceiling):
    return True if min(indices) > 0 and max(indices) <= ceiling else False

def selectPolygonTypesAsResedentials(polygon_types):
    showAvailablePolygonTypes(polygon_types)
    doPrint("Switch to console to provide your choice and proceed.",True, False, inLogFile=False)
    while(True):
        strIndices = input("Select polygons to be treated as residentials (Space Separated) : ")
        arrIndices = np.array(np.array(strIndices.split(" ")), dtype=int)
        if isSelectionValid(arrIndices, len(polygon_types.keys())):
            selected_polygon_types = [list(polygon_types.keys())[i-1] for i in arrIndices]
            # print(selected_polygon_types)
            doPrint("Polygons types under consideration : {}".format(selected_polygon_types))
            return selected_polygon_types
        else:
            print ("Choice out of range! Try again")

def ExtractRequiredPolygons(polygon_types):
    polygons = []
    points = []
    polygon_ids = traci.polygon.getIDList()
    doPrint("Extracting polygons of selected types..")
    for polygon_id in polygon_ids:
        if traci.polygon.getType(polygon_id) in polygon_types:
            polygons.append(Polygon(polygon_id, traci.polygon.getShape(polygon_id)))
            points.extend(polygons[-1].corner_points)
            traci.polygon.setColor(polygon_id, [0, 255, 0]) # Green
    doPrint("Total {} Polygons are considered!".format(len(polygons)))
    return polygons, points

def sortAllPoints(points: list[Point]):
    points.sort(key=lambda pt: pt.y )
    points.sort(key=lambda pt: pt.x )

def showAllPoints(points: list[Point]):
    i = 1
    for pt in points:
        print(i),
        pt.print()
        i+=1
        
def readStaticNetwork(sumocfg, alongwithInternal=False) -> sumolib.net.Net:
    netFile = sumolib.xml.parse(sumocfg, 'net-file')
    netFileName = list(netFile)[0].__getattribute__('value')
    return sumolib.net.readNet(netFileName, withInternal=alongwithInternal)

def getVehicleTypes():
    vTypes = []
    for typ in traci.vehicletype.getIDList():
        vTypes.append(traci.vehicletype.getVehicleClass(typ))
    return vTypes

def showVehicleTypes(vTypes):
    i = 1
    for typ in vTypes:
        print("{}. {}".format(i, typ))
        i += 1

# def getRealRoads():
def selectVehicleTypesForParcelCarry():
    vehicleTypes = getVehicleTypes()
    showVehicleTypes(vehicleTypes)
    while(True):
        strIndices = input("Select vehicles to carry parels (Space Separated) : ")
        arrIndices = np.array(np.array(strIndices.split(" ")), dtype=int)
        if isSelectionValid(arrIndices, len(vehicleTypes)):
            selected_vehicle_types = [vehicleTypes[i-1] for i in arrIndices]
            doPrint("Vehicle types selected for parcel carrying : {}".format(selected_vehicle_types))
            return selected_vehicle_types
        else:
            print ("Choice out of range! Try again")

def isAllowedEdge(edge, vehicleTypes):
    allowed = False
    for typ in vehicleTypes:
        allowed |= edge.allows(typ)
    # return True
    return allowed

def getAllowedRoad(net, vehicleTypes) -> list[Road]:
    # sumolib.net.edge.Edge.allows()
    edges = net.getEdges(False)
    roads = []
    for edge in edges:
        if isAllowedEdge(edge, vehicleTypes):
            width = 0
            for lane in edge.getLanes():
                width += lane.getWidth()
            # print(edge.getShape())
            # print(type(edge.getShape()[0][0]))
            roads.append(Road(edge.getID(), width, edge.getShape()))
    return roads    

def getNearestCentralLinePointIndex(line: list[tuple], midPoint: Point):
    min = None
    minIndex = None
    for i in range(len(line)):
        distance = midPoint.getDistanceFrom(Point(line[i][0], line[i][1]))
        min, minIndex = (distance, i) if min is None or distance < min else (min, minIndex)
    return minIndex

def getProjectionPoint(pt1: Point, pt2: Point, outPt: Point) -> Point:
    m = (pt2.y - pt1.y) / (pt2.x - pt1.x)
    c = pt1.y - m * pt1.x
    d = outPt.y + m * outPt.x
    a = (d - c) / (2 * m)
    b = m * a + c
    return Point(a, b)

def addWarehouse(net: sumolib.net.Net, middle = False):
    boundary = net.getBoundary()
    xmid = (boundary[0] + boundary[2]) / 2 if middle else random.uniform(boundary[0], boundary[2])
    ymid = (boundary[1] + boundary[3]) / 2 if middle else random.uniform(boundary[1], boundary[3])
    midPoint = Point(xmid, ymid)
    # traci.polygon.add("warehouse", [ (xmid-2, ymid-2), (xmid+2, ymid-2), (xmid+2, ymid+2), (xmid-2, ymid+2)], (0, 0, 255), True, "Warehouse",1)
    road = traci.simulation.convertRoad(xmid, ymid, False, "passenger")
    doPrint(road)
    edge = net.getEdge(road[0])
    center_line = edge.getShape()
    ptIndex = getNearestCentralLinePointIndex(center_line, midPoint)
    if ptIndex == 0:
        projPoint = getProjectionPoint(Point(center_line[0][0], center_line[0][1]), Point(center_line[1][0], center_line[1][1]), midPoint)
    elif ptIndex == len(center_line)-1:
        projPoint = getProjectionPoint(Point(center_line[ptIndex][0], center_line[ptIndex][1]), Point(center_line[ptIndex-1][0], center_line[ptIndex-1][1]), midPoint)       
    else:
        projPoint1 = getProjectionPoint(Point(center_line[ptIndex][0], center_line[ptIndex][1]), Point(center_line[ptIndex-1][0], center_line[ptIndex-1][1]), midPoint)       
        projPoint2 = getProjectionPoint(Point(center_line[ptIndex][0], center_line[ptIndex][1]), Point(center_line[ptIndex+1][0], center_line[ptIndex+1][1]), midPoint)       
        projPoint = projPoint1 if projPoint1.getDistanceFrom(midPoint) < projPoint2.getDistanceFrom(midPoint) else projPoint2
    distance = projPoint.getDistanceFrom(midPoint)
    width = 4 # for warehouse width
    for lane in edge.getLanes():
        width += lane.getWidth()
    xWH = ((distance - width) * projPoint.x + width * midPoint.x)/distance
    yWH = ((distance - width) * projPoint.y + width * midPoint.y)/distance
    traci.polygon.add("warehouse", [ (xWH-2, yWH-2), (xWH+2, yWH-2), (xWH+2, yWH+2), (xWH-2, yWH+2)], (0, 0, 255), True, "Warehouse")
    print(edge.getFromNode().getCoord())
    print(edge.getToNode().getCoord())
    # lane = net.getLane(road[0]+"_"+str(road[2]))
    # doPrint(traci.lane.getLinks(lane.getID()))
    # links = traci.lane.getLinks(lane.getID())
    # visited = []
    # for link in links:
    #     doPrint(link)
    #     newLinks = traci.lane.getLinks(link[0])
    #     for nlnk in newLinks:
    #         if links.count(nlnk) == 0:
    #             links.append(nlnk)
    

def InitializeAllPossibleStations(roads : list[Road]) -> list[Station]:
    doPrint("Initializing all possbile stations at {} meters apart!".format(Constant.INITIAL_STATION_PLACEMENT_DISTANCE))
    stations = []
    for road in roads:
        locations = road.getAllPossibleStationLocation()
        for location in locations:
            station = Station(location, road)
            stations.append(station)
            doPrint("Station {} is added at ({}, {})".format(station.id, station.location.x, station.location.y))
    return stations

def calculateCoverageByPossibleStations(stations: list[Station], points: list[CornerPoint]):
    for station in stations:
        # print("\r Calculating Coverage by stations {} out of {}...".format(station.id, len(stations)),end="")
        doPrint("\r Calculating Coverage by stations {} out of {}...".format(station.id, len(stations)),end="", inLogFile=False)
        station.findAllPointsInRange(points)
        station.determineCoveredPolygon()
    doPrint("Calculating coverage by all stations...Done!")

def showStations(stations : list[Station]):
    for station in stations:
        # station.printStation()
        traci.poi.add(station.id, station.location.x, station.location.y, (255, 0, 0), "station")
        # traci.poi.add(station.id, station.location.x, station.location.y, (0, 0, 0, 0), "S-"+str(station.id))

def reportUncoveredPolygon(polygons: list[Polygon]) :
    uncovered_count = 0
    for polygon in polygons:
        # print("\r Checking polygon coverage {} out of {}...".format(polygons.index(polygon)+1, len(polygons)),end="")
        doPrint("\r Checking polygon coverage {} out of {}...".format(polygons.index(polygon)+1, len(polygons)),end="", inLogFile=False)
        if not polygon.isFullyCovered():
            # print(polygon.id, polygon.corner_points)
            doPrint("Polygon {} of shape {} is not covered!".format(polygon.id, polygon.shape_string))
            uncovered_count += 1
    if uncovered_count == 0:
        doPrint("\n All polygons are covered!")
    else:
        doPrint("\n {} out of {} Polygons are not covered!".format(uncovered_count, len(polygons)))
    return uncovered_count

def getCoverageMatrix(polygons: list[Polygon], stations: list[Station]):
    cov = np.zeros((len(stations), len(polygons)))
    for i in range(len(polygons)):
        for station in polygons[i].fully_covered_by:
            cov[station.id-1, i] = 1
    return cov

def formulateProblem(stationCount, polygonCount, coverageMatrix):
    doPrint("Formulating problem in PuLP!")
    problem = pl.LpProblem("Minimum_Coverage", pl.LpMinimize)
    x = pl.LpVariable.dicts("x", [i for i in range(stationCount)], lowBound=0.0, cat=pl.LpBinary)
    problem += pl.lpSum([x[i] for i in range(stationCount)])
    for j in range(polygonCount):
        problem += pl.lpSum([x[i] * coverageMatrix[i, j] for i in range(stationCount)]) >= 1.0
    return problem, x

def removeStation(allStations : list[Station], index: int):
    station = allStations[index]
    station.prepareForRemoval()
    traci.poi.remove(station.id)
    allStations.remove(station)
    doPrint("Station {} at ({}, {}) is removed.".format(station.id, station.location.x, station.location.y))

def reportPolygonCoverage(polygons : list[Polygon]):
    for polygon in polygons:
        doPrint("The polygon {} is covered by station(s) {}.".format(polygon.id, polygon.getCoveringStationIds()))

def doPrint(msg, inSUMO = True, inConsole = True, end="\n", inLogFile = True ):
    if inConsole:
        print(msg, end=end)
    if inSUMO:
        traci.simulation.writeMessage(msg)
    if inLogFile:
        Logger.Write(str(msg))


def runMapScanner(config_file_path, gui=False, logFile="log.txt"):
    if gui:
        sumoCmd = ["sumo-gui", "-c", config_file_path]
    else:
        sumoCmd = ["sumo", "-c", config_file_path]
    traci.start(sumoCmd)
    net = readStaticNetwork(config_file_path)
    Logger.Initialize(logFile)
    addWarehouse(net, True)

    # # doPrint("Click on the Run button above to start the simulation.", inConsole=False, inLogFile=False)
    # # ** Don't delele!! Below two lines of code is for generic selection of poly type
    # # polygon_types = getTypewisePolygonCount()
    # # residential_polygon_types = selectPolygonTypesAsResedentials(polygon_types)
    # residential_polygon_types = ['building']
    # residential_polygons, all_points = ExtractRequiredPolygons(residential_polygon_types)
    # sortAllPoints(all_points)
    # # vehicleTypes = selectVehicleTypesForParcelCarry()     
    # vehicleTypes = ['taxi']
    # roads = getAllowedRoad(net, vehicleTypes)
    # possible_stations = InitializeAllPossibleStations(roads)
    # showStations(possible_stations)
    # calculateCoverageByPossibleStations(possible_stations, all_points)
    # if reportUncoveredPolygon(residential_polygons) == 0:
    #     coverage_matrix = getCoverageMatrix(residential_polygons, possible_stations)
    #     problem, x = formulateProblem(len(possible_stations), len(residential_polygons), coverage_matrix)    
    #     solver = pl.PULP_CBC_CMD()
    #     doPrint("Solving Problem using PuLP!")
    #     if(problem.solve(solver) == 1):
    #         doPrint("PuLP has solve with objective value {}. ".format(problem.objective.value()))
    #         doPrint("Removing unselected stations...")
    #         for i in range(len(x)-1, -1, -1):
    #             # print(x[i].value())
    #             if x[i].value() == 0:
    #                 removeStation(possible_stations, i)
    #         doPrint("Optimal number of Stations after removal : {}".format(Station.count), True)                
    #         for station in possible_stations:
    #             # station.printStation()
    #             doPrint("Station id : {}, location = ({}, {}), road = {}".format(station.id, station.location.x, station.location.y, station.at_road.id))
    #         doPrint("Cross-verifying coverage..")            
    #         reportUncoveredPolygon(residential_polygons)
    #         reportPolygonCoverage(residential_polygons)
    #     else:
    #         doPrint(pl.LpStatus[problem.status])
        
    # nodes = net.getNodes()
    # for node in nodes:
    #     print("ID : ", node.getID(), ", Incoming : ", end="" )
    #     incoming = node.getIncoming()
    #     for i in incoming:
    #         print(i.getID(), end=", ")
    #     outgoing = node.getOutgoing()
    #     print("Outgoing : ", end="")
    #     for o in outgoing:
    #         print(o.getID(), end=", ")
    #     print()

    Logger.Close()
    traci.close()

runMapScanner("test.sumocfg.xml", True)
