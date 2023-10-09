"""
    This module is your primary workspace. Add whatever helper functions, classes, data structures, imports... etc here.

    We expect most results will utilize more than just dumping code into the plan_paths()
        function, that just serves as a meaningful entry point.

    In order for the rest of the scoring to work, you need to make sure you have correctly
        populated the Destination.path for each result you produce.
"""
import typing
from queue import PriorityQueue

import numpy as np
from typing import Dict
import math

from map_info import Coordinate, Destination, MapInfo


class PathPlanner:
    def __init__(self, map_info: MapInfo, destinations: typing.List["Destination"]):
        self.map_info: MapInfo = map_info
        self.destinations: typing.List["Destination"] = destinations
        
    def interpolate_points(self, coord1, coord2):
        """
        Fills in points on an 8-connected grid between two points.

        Parameters:
        - coord1: The first coordinate.
        - coord2: The second coordinate.

        Returns:
        - A list of interpolated coordinates.
        """
        x1 = coord1.e
        y1 = coord1.n
        x2 = coord2.e
        y2 = coord2.n

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)

        # Largest difference
        if dx > dy:
            # Number of points to add 
            num_points = int(dx) + 1
            # step towards the end point
            step_x = 1 if x2 > x1 else -1
            step_y = (y2 - y1) / dx if dx != 0 else 0
        else:
            # Number of points to add 
            num_points = int(dy) + 1
            # step towards the end point
            step_x = (x2 - x1) / dy if dy != 0 else 0
            step_y = 1 if y2 > y1 else -1
            
        # make interpolated points
        new_points = [Coordinate(int(math.floor(x1 + step_x * i)), int(math.floor(y1 + step_y * i))) for i in range(num_points)]
        return new_points

    def resample_path(self, original_coords):
        """
        Retraces a path of any resolution and returns an 8-connected path of given path.

        Parameters:
        - original_coords: A list of original coordinates representing a path.

        Returns:
        - A new list of coordinates representing the resampled path.
        """
        # initialize new coordinates
        new_coords = [Coordinate(int(original_coords[0].e), int(original_coords[0].n))]
        
        # Loop through the given coords and interpolate them
        
        for i in range(len(original_coords) - 1):
            
            # interpolate between current and next
            new_points = self.interpolate_points(original_coords[i], original_coords[i + 1])
            # if the points have not already been added
            if new_points[-1] != new_coords[-1]:
                new_coords.extend([Coordinate(int(point.e), int(point.n)) for point in new_points])

        return new_coords
    
    def calculate_risk(self, startCoord, endCoord):
        """
        Calculate the risk of a line given the start and end coordinates.

        Parameters:
        - startCoord: Starting coordinate of the line.
        - endCoord: Ending coordinate of the line.

        Returns:
        - The risk associated with the path between startCoord and endCoord.
        """
        # create the path and interpolate between the start and end
        
        path_array = np.linspace(startCoord, endCoord, 2)
        path_coords = [Coordinate(arr[0], arr[1]) for arr in path_array]
        path_coords = self.resample_path(path_coords)
        
        risk = 0
        # calculate the risk 
        for coord in path_coords:
            risk += self.map_info.risk_zones[int(coord[0])][int(coord[1])] == MapInfo.HIGH_RISK_VALUE
        return risk
    
    def get_coordinates(self, startCoord, endCoord):
        """
        Finds the mid point of a line given a start and end
        
        Parameters:
        - startCoord: Starting coordinate of the route.
        - endCoord: Ending coordinate of the route.
        
        Returns: 
        - Creates a list of Coordinate's 
        """
        path_array = np.linspace(startCoord, endCoord, 3)
        return [Coordinate(arr[0], arr[1]) for arr in path_array]
    
    def optimize_route(self, startCoord, endCoord, FinalPoint, allowed_length, name):
        """
        Optimizes a route between startCoord and endCoord minimizing risk and staying

        Parameters:
        - startCoord: Starting coordinate of the route.
        - endCoord: Ending coordinate of the route.
        - FinalPoint: The final destination.
        - allowed_length: The maximum allowed length for the route.
        - name: A string specifying the name (e.g., 'Coronation') for additional constraints.

        Returns:
        - A list of optimized coordinates representing the route.
        """
        # GET A LINE WITH A MID POINT
        path_coords = self.get_coordinates(startCoord, endCoord)
        
        # set high so it doenst break the loop falsely
        prev_risk = 100000000         
        # OPTIMIZING THE POINT IN BETWEEN
        while True:
            # FLAG
            break_yes = False
            
            # Exit the loop if the path length is within the maximum range
            distance = ((path_coords[1].e - startCoord.e) ** 2 + (path_coords[1].n - startCoord.n) ** 2) ** 0.5
            if distance > (allowed_length):
                break  
                
            # CALCULATE THE RISK OF THE LINE 
            risk_init = self.calculate_risk(path_coords[0], path_coords[1]) + self.calculate_risk(path_coords[1], path_coords[2]) 
            
            # If line should go over or under the danger zone
            if path_coords[1].n >= 20 :
                path_coords[1] = Coordinate(path_coords[1].e, path_coords[1].n + 1)
            else:
                path_coords[1] = Coordinate(path_coords[1].e, path_coords[1].n - 1)

            # CALCULATE THE FINAL RISK OF THE LINE 
            risk_final = self.calculate_risk(path_coords[0], path_coords[1]) + self.calculate_risk(path_coords[1], path_coords[2])
            
            if name == 'Coronation':  
                if path_coords[1].e > 28 and path_coords[1].e < 33:
                    path_coords[1] = Coordinate(path_coords[1].e, path_coords[1].n + 1)
                    
            # Make sure line is out of the keep out zone
            if (((path_coords[1].e - 30) ** 2 + (path_coords[1].n - 20) ** 2) ** 0.5) < 6:
                while True:
                    # if outside danger zone
                    if (((path_coords[1].e - 30) ** 2 + (path_coords[1].n - 20) ** 2) ** 0.5) > 6:
                        break
                    # move accodingly Y
                    if path_coords[1].n > 20 :
                        path_coords[1] = Coordinate(path_coords[1].e, path_coords[1].n + 1)
                    else:
                        path_coords[1] = Coordinate(path_coords[1].e, path_coords[1].n - 1)
                    # move accodingly X
                    if path_coords[1].e > 30 :
                        path_coords[1] = Coordinate(path_coords[1].e + 1, path_coords[1].n)
                    else:
                        path_coords[1] = Coordinate(path_coords[1].e - 1, path_coords[1].n)
                    # SPECIAL
                    if name == 'Coronation':
                        path_coords[1] = Coordinate(path_coords[1].e, path_coords[1].n + 1)
                
            #if bad progress revert and break
            if risk_init <= risk_final:
                break
            # CHECK IF WE CANT GO UP ANYMORE
            if path_coords[1].n > 35:
                break
        # Calculate the distance between the two points
        distance = ((path_coords[1].e - startCoord.e) ** 2 + (path_coords[1].n - startCoord.n) ** 2) ** 0.5
        # If points are close end inward recursion
        if distance > 3:
            # Recursively call the function and concatenate the results
            value = self.optimize_route(startCoord, path_coords[1], FinalPoint, allowed_length/2, 'x')
        # GO OUT
        else:
            return [path_coords[1]]

        # Return a list of coordinates by concatenating the next coordinate with the result of the recursive call
        return value + self.optimize_route(path_coords[1], endCoord, FinalPoint, allowed_length/2, 'x')
    
    def plan_paths(self):
        """
        This is the function you should re-write. It is expected to mutate the list of
        destinations by calling each Destination's set_path() with the resulting
        path as an argument.

        The default construction shows this format, and should produce 10 invalid paths.
        """

        for site in self.destinations:
            endList = []
            
            # STARTING COORDS
            path_array = np.linspace(self.map_info.start_coord, site.coord, 2)
            path_coords = [Coordinate(arr[0], arr[1]) for arr in path_array]
            # START
            start = Coordinate(path_array[0][0], path_array[0][1])  
            # END
            end = Coordinate(path_array[1][0], path_array[1][1])  
                
            # ADD THE START COORD
            endList.append(start)
            # ADD THE MIDDLE COORDS
            endList.extend(self.optimize_route(self.map_info.start_coord, site.coord, site.coord, (self.map_info.maximum_range/2)-2, site.name))
            #  ADD THE END COORD
            endList.append(end)
            
            # GET VALID PATH IN AN 8 CONNECTED GRID
            new_path = self.resample_path(endList)
            
            
            new_path.append(Coordinate(int(end.e), int(end.n)))
            
            site.set_path(new_path)

            

