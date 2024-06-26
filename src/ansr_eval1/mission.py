import numpy as np
import json
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point, LineString

class Car:

    def __init__(self, id, priority):
        self.id = id
        self.priority = priority
        self.map = None

class Region:

    def __init__(self, probability, polygon):
        self.probability = probability
        self.polygon = polygon
        self.cells = None

class Zone:

    def __init__(self, polygon, t_start=None, t_end=None):
        self.polygon = polygon
        self.t_start = t_start
        self.t_end = t_end

class Cell:

    def __init__(self, polygon):
        self.polygon = polygon
        self.in_keep_out_zone = False
        self.keep_out_zone = None
        self.in_any_region = False
        self.visited = False

class Route:

    def __init__(self, points_list, rectangles):
        self.points_list = points_list
        self.rectangles = rectangles

def carSortCriterion(car):
	return car.priority

def polygonSortCriterion(polygon):
	return polygon.probability

def cellsInKeepOutZone(cells, zone):
        cell_list = []
        cell_idx = 0
        for cell in cells:
            if cell.polygon.intersects(zone.polygon):
                cell_list.append(cell_idx)
            cell_idx = cell_idx + 1
        return cell_list

def cellInKeepOutZone(cell, zone):
        return (cell.polygon.intersects(zone.polygon))

def cellInAnyKeepOutZone(cell, zones):
        for zone in zones:
            if (cell.polygon.intersects(zone.polygon)):
                return True, zone
        return False, None

def cellsInRegion(cells, region):
        cell_list = []
        cell_idx = 0
        for cell in cells:
            if cell.polygon.intersects(region.polygon):
                cell_list.append(cell_idx)
            cell_idx = cell_idx + 1
        return cell_list

def pointWithinPolygon(x, y, polygon):
        distance = Point(x, y).distance(polygon)
        if distance == 0:
            return True
        return False

def sort_points_by_distance(points, source):
        #points.sort(key = lambda p: (p.x - x)**2 + (p.y - y)**2)
        points.sort(key = lambda p: (p[0] - source[0])**2 + (p[1] - source[1])**2)
        return points


class Mission:

    def __init__(self, descriptionFile, configFile):
        self.car_list = []
        self.keep_out_zones = []
        self.route_list = []
        self.rect_list = []
        self.cells = []
        self.AOI_list = []

        f_description = open(descriptionFile)
        self.description_data = json.load(f_description)

        f_config = open(configFile)
        self.config_data = json.load(f_config)

        cell_0 = Cell(Polygon([(-128, 121), (-128, 135), (0,135), (0,121)]))
        cell_1 = Cell(Polygon([(0, 121), (0, 135), (128,135), (128,121)]))
        cell_2 = Cell(Polygon([(-7, 76.8), (-7, 128), (7,128), (7,76.8)]))
        cell_3 = Cell(Polygon([(-128, 70), (-128, 83.8), (0,83.8), (0,70)]))
        cell_4 = Cell(Polygon([(-135, 76.8), (-121, 76.8), (-121,0), (-135,0)]))
        cell_5 = Cell(Polygon([(-7, 76.8), (-7, 0), (7,0), (7,76.8)]))
        cell_6 = Cell(Polygon([(-128, -7), (-128, 7), (0,7), (0,-7)]))
        cell_7 = Cell(Polygon([(0, -7), (0, 7), (128,7), (128,-7)]))
        cell_8 = Cell(Polygon([(-135, 0), (-121, 0), (-121,-128), (-135,-128)]))
        cell_9 = Cell(Polygon([(-7, -128), (7, -128), (7,0), (-7,0)]))
        cell_10 = Cell(Polygon([(135, -128), (121, -128), (121,0), (135,0)]))
        cell_11 = Cell(Polygon([(0, -135), (0, -121), (128,-121), (128,-135)]))
        cell_12 = Cell(Polygon([(0, -121), (0, -135), (-128,-135), (-128,-121)]))
        cell_13 = Cell(Polygon([(-135, 128), (-121, 128), (-121,76.8), (-135,76.8)]))
        cell_14 = Cell(Polygon([(135, 0), (121, 0), (121,128), (135,128)]))

        self.cells.append(cell_0)
        self.cells.append(cell_1)
        self.cells.append(cell_2)
        self.cells.append(cell_3)
        self.cells.append(cell_4)
        self.cells.append(cell_5)
        self.cells.append(cell_6)
        self.cells.append(cell_7)
        self.cells.append(cell_8)
        self.cells.append(cell_9)
        self.cells.append(cell_10)
        self.cells.append(cell_11)
        self.cells.append(cell_12)
        self.cells.append(cell_13)
        self.cells.append(cell_14)

        # Graph data structure for path computation
        self.graph = None
        
        self.parse()

    def parse(self):
        self.scenario_id = self.description_data["scenario_id"]
        self.mission_class = self.description_data["mission_class"]

        self.start_airsim_state = self.config_data["controllable_vehicle_start_loc"]

        if "areas_of_interest" in self.description_data["scenario_objective"]:
            for area in self.description_data["scenario_objective"]["areas_of_interest"]:
                for polygon in area["polygon_vertices"]:
                    enu_points = []
                    for point in polygon:
                        enu_point = point[::-1]
                        enu_points.append(enu_point)
                    poly = Polygon(enu_points)
                    self.AOI_list.append(poly)


        if "entities_of_interest" in self.description_data["scenario_objective"]:
            for entity in self.description_data["scenario_objective"]["entities_of_interest"]:
                id = entity["entity_id"]
                priority = entity["priority"]
                car = Car(id, priority)

                region_list = []
                total_prob = 0
                if "entity_priors" in entity:
                    if "location_belief_map" in entity["entity_priors"]:
                        for area in entity["entity_priors"]["location_belief_map"]:
                            for polygon in area["polygon_vertices"]:
                                enu_points = []
                                for point in polygon:
                                    enu_point = point[::-1]
                                    enu_points.append(enu_point)
                                poly = Polygon(enu_points)
                                prob = area["probability"]
                                total_prob = total_prob + prob
                                region = Region(prob, poly)
                                region_list.append(region)
                        # AOI
                        if total_prob < 1:
                            for AOI in self.AOI_list:
                                diff = AOI
                                for region in region_list:
                                    diff = diff - region.polygon
                                region = Region(1-total_prob, diff)
                                region_list.append(region)

                        region_list.sort(reverse=True, key=polygonSortCriterion)
                        car.map = region_list
                self.car_list.append(car)

            self.car_list.sort(key=carSortCriterion)

        if "spatial_constraints" in self.description_data["scenario_constraints"]:
            if "keep_out_zones" in self.description_data["scenario_constraints"]["spatial_constraints"]:
                for keep_out_zone in self.description_data["scenario_constraints"]["spatial_constraints"]["keep_out_zones"]:
                    enu_points = []
                    for point in  keep_out_zone["keep_out_polygon_vertices"]:
                        enu_point = point[::-1]
                        enu_points.append(enu_point)
                    poly = Polygon(enu_points)
                    t_start = keep_out_zone["no_earlier_than"]
                    t_end = keep_out_zone["no_later_than"]
                    zone = Zone(poly, t_start, t_end)
                    cells = cellsInKeepOutZone(self.cells, zone)
                    zone.cells = cells
                    self.keep_out_zones.append(zone)


        if "routes_of_interest" in self.description_data["scenario_objective"]:
            for route_data in self.description_data["scenario_objective"]["routes_of_interest"]:
                points_list = []
                for route_points in route_data["route_points"]:
                    enu_points = []
                    for point in route_points:
                        enu_point = point[::-1]
                        enu_points.append(enu_point)
                    points_list.append(enu_points)

                rectangles = []
                for rectangle in route_data["enclosing_rectangles"]:
                    enu_points = []
                    for point in rectangle:
                        enu_point = point[::-1]
                        enu_points.append(enu_point)
                    rect = Polygon(enu_points)
                    rectangles.append(rect)
    
                route = Route(points_list, rectangles)
                self.route_list.append(route)

        # Postprocessing
        for car in self.car_list:
            for region in car.map:
                cell_list = cellsInRegion(self.cells,region)
                region.cells = cell_list

        for cell in self.cells:
            bool, zone = cellInAnyKeepOutZone(cell, self.keep_out_zones)
            if bool:
                cell.in_keep_out_zone = True
                cell.keep_out_zone = zone
            if self.cellInAnyRegion(cell):
                cell.in_any_region = True

        
    def cellInAnyRegion(self, cell):
        for car in self.car_list:
            for region in car.map:
                if cell.polygon.intersects(region.polygon):
                    return True
        return False


    def getSpecialAOIPoints(self):
        points = ([64.0, 68.0], [-55.0, -64.0], [64.0, -60.0])
        AOI_points = []
        for car in self.car_list:
            for region in car.map:
                poly = region.polygon
                for point in points:
                    p = Point(point)
                    if poly.contains(p) or poly.exterior.distance(p) < 20:
                        if point not in AOI_points:
                            AOI_points.append(point)
        return AOI_points


    def getSpecialAOIPointEntry(self, AOI_point, ideal_point):
        x = ideal_point[0]
        y = ideal_point[1]
        dx_array = [64, -64]
        dy_array = [64, -64]
        points = []
        points.append(AOI_point)
        for dx in dx_array:
            for dy in dy_array:
                intersect = False
                x1 = x + dx
                y1 = y + dy
                points.append([x1,y1])
                #print(points)
                linestring = LineString(points)
                for zone in self.keep_out_zones:
                    if linestring.intersects(zone.polygon):
                        intersect = True
                        break
                if intersect:
                    points.pop()
                    #print(points)
                else:
                    return [x1, y1]
        return None

    def getRouteEntry(self, route, source_point):
        # TODO
        points_list = route.points_list[0]
        route_points = points_list[:]
        sorted_route_points = sort_points_by_distance(route_points, source_point)

        points = []
        points.append(source_point)
        for route_point in sorted_route_points:
                intersect = False
                points.append(route_point)
                linestring = LineString(points)
                for zone in self.keep_out_zones:
                    if linestring.intersects(zone.polygon):
                        intersect = True
                        break
                if intersect:
                    points.pop()
                else:
                    return route_point
        return None
    
    def interpolate_route_waypoints(self, route):
        interpolated_waypoint_list= []
        points_list = route.points_list[0]
        for route_point in points_list:
            waypoint = [route_point[0], route_point[1]] 
            interpolated_waypoint_list.append(waypoint)
        return interpolated_waypoint_list

    def get_route_waypoint_list(self, route_point_list, route_entry):
        route_waypoint_list = []
        head_list = []
        for waypoint in route_point_list:
            # print(waypoint)
            if  waypoint != route_entry:
                head_list.append(waypoint)
            else:
                break

        #print('head', head_list)
        tail_list = [item for item in route_point_list if item not in head_list]
        #print('tail', tail_list)

        head_list_reverse = head_list[:]
        head_list_reverse.reverse()

        tail_list_reverse = tail_list[:]
        tail_list_reverse.reverse()

        if len(head_list) != 0: head_list.pop(0)
        if len(tail_list_reverse) != 0: tail_list_reverse.pop(0)

        route_waypoint_list.extend(tail_list)
        route_waypoint_list.extend(tail_list_reverse)
        route_waypoint_list.extend(head_list_reverse)
        route_waypoint_list.extend(head_list)

        return route_waypoint_list

if __name__ == "__main__":
        # mission = Mission('../../../mission-schema/examples/Maneuver/RouteSearch/RSM002/description.json', '../../../mission-schema/examples/Maneuver/RouteSearch/RSM002/config.json') 
        # mission = Mission('../../../mission-schema/examples/Maneuver/AreaSearch/ASM004/description.json', '../../../mission-schema/examples/Maneuver/AreaSearch/ASM004/config.json') 
        mission = Mission('../../../adk/mission_briefing/description.json', '../../../adk/mission_briefing/config.json') 

        for car in mission.car_list:
            print(car.id)
            print('priority: ', car.priority)
            for region in car.map:
                print('probability: ',region.probability)
                print(region.polygon)
                print('cells: ', region.cells)
                x,y = region.polygon.exterior.xy
                plt.plot(x,y, 'green')
    
        for area in mission.AOI_list:
            print('AOI ', area)
            x,y = area.exterior.xy
            plt.plot(x,y, 'purple')

        for zone in mission.keep_out_zones:
            print('Keep Out Zone', zone.polygon)
            print("t_start \t", zone.t_start)
            print("t_end \t\t", zone.t_end)
            x,y = zone.polygon.exterior.xy
            plt.plot(x,y, 'red')

        cell_idx = 0
        for cell in mission.cells:
            print('cell ', cell_idx)
            print('in_keep_out_zone \t', cell.in_keep_out_zone)
            print('in_any_region \t\t', cell.in_any_region)
            x,y = cell.polygon.exterior.xy
            plt.plot(x,y, "black")
            cell_idx = cell_idx +1

        for route in mission.route_list:
            for points in route.points_list:
                linestring = LineString(points)
                plt.plot(*linestring.xy,  linewidth=7.0, color="blue")

            for rect in route.rectangles:
                x,y = rect.exterior.xy
                #plt.plot(x,y)
    
        plt.plot(mission.start_airsim_state[1],mission.start_airsim_state[0], 'r*', markersize=10)


        plt.show()

        points = mission.getSpecialAOIPoints()
        for point in points:
            print('AOI point', point)
            entry = mission.getSpecialAOIPointEntry(point)
            print('Entry', entry)

        if (mission.mission_class == 'Route Search'):
            points = mission.route_list[0].points_list[0]
            point_list = points[:] # make a copy
            print('points', points)
            source = [0, 0]
            print(sort_points_by_distance(point_list, source))
            print(mission.getRouteEntry(mission.route_list[0], source))
            source = [-200, -200]
            print(sort_points_by_distance(point_list, source))
            print(mission.getRouteEntry(mission.route_list[0], source))
            source = [-100, 90]
            print(sort_points_by_distance(point_list, source))
            print(mission.getRouteEntry(mission.route_list[0], source))
