"""
Rapidly Exploring Random Trees (RRT) Path Planning

This module implements the RRT algorithm for path planning in a 2D environment.

Author: Ghassen Hafsia
Date: 12/27/2023

...
"""

import random
import math 
import pygame

class RRTMap:
    """
        RRTMap class handles the visualization of the map and obstacles.

        Attributes:
        - start: Tuple representing the start position.
        - goal: Tuple representing the goal position.
        - MapDimensions: Tuple representing the dimensions of the map.
        - obsdim: Dimension of obstacles.
        - obstacles: List storing obstacle positions.

        Methods:
        - __init__: Constructor method to initialize the map.
        - drawMap: Method to draw the map with obstacles, start, and goal positions.
        - drawPath: Method to draw the original RRT path.
        - drawobs: Method to draw obstacles on the map.
        """ 
    def __init__(self,start,goal,mapDimensions,obsdim,obstacles,prohibited_zone):
        """
        Initialize RRTMap with start and goal positions, map dimensions, obstacle dimensions, and the number of obstacles.

        Parameters:
        - start (tuple): Tuple representing the start position.
        - goal (tuple): Tuple representing the goal position.
        - map_dimensions (tuple): Tuple representing the dimensions of the map.
        - obsdim (int): Dimension of obstacles.
        - obsnum (int): Number of obstacles.
        """
        self.start=start
        self.goal=goal
        self.MapDimensions = mapDimensions
        self.Mapw,self.Maph=self.MapDimensions

        #window settings
        self.MapWindowName='RRT path planning' 
        pygame.display.set_caption(self.MapWindowName)
        self.map=pygame.display.set_mode((self.Mapw,self.Maph))
        self.map.fill ((255,255,255))
        self.nodeRad=3
        self.nodeThickness=0
        self.edgeThickness=1

        self.obstacles=obstacles
        self.obsdim = obsdim
        self.prohibited_zone = prohibited_zone
        #Colors
        self.grey = (70,70,70)
        self.blue = (0,0,255)
        self.green = (0,255,0)
        self.red = (255,0,0)
        self.white = (255,255,255)
        self.black = (0,0,0)




    def drawMap(self):
        """
        Draw the map with obstacles, start, and goal positions.

        Parameters:
        - obstacles (list): List of obstacle positions.
        """
               
        pygame.draw.circle(self.map,self.green,self.start,self.nodeRad+10,1)
        pygame.draw.circle(self.map,self.grey,self.goal,self.nodeRad+10,1)
        rect_width = abs(self.prohibited_zone[2] - self.prohibited_zone[0])
        rect_height = abs(self.prohibited_zone[3] - self.prohibited_zone[1])        
        rectangle = pygame.Rect(self.prohibited_zone[0], self.prohibited_zone[1], rect_width, rect_height)
        pygame.draw.rect(self.map, self.red, rectangle)
        self.drawobs()

    def drawPath(self, path, path_smoothed): 
        """
        Draw the original RRT path.
        Draw the optimized (smoothed) RRT path.

        Parameters:
        - path (list): List of nodes representing the original RRT path.
        - path_smoothed (list): List of nodes representing the optimized RRT path.
        """    
        for node in path:
            pygame.draw.circle(self.map, self.blue, node, self.nodeRad+5, 0)
        for node in path_smoothed:
            pygame.draw.circle(self.map, self.red, node, self.nodeRad+5, 0)      

    def drawPath_1(self, path): 
        """
        Draw the original RRT path.
        Draw the optimized (smoothed) RRT path.

        Parameters:
        - path (list): List of nodes representing the original RRT path.
        - path_smoothed (list): List of nodes representing the optimized RRT path.
        """    
        for node in path:
            pygame.draw.circle(self.map, self.blue, node, self.nodeRad+5, 0)
     

    def drawobs(self): 
        """
        Draw obstacles on the map.

        Parameters:
        - obstacles (list): List of obstacle positions.
        """     
        obstaclesList=self.obstacles.copy()
        while (len(obstaclesList)>0):
            obstacle = obstaclesList.pop(0)
            pygame.draw.circle(self.map, self.black, obstacle, self.obsdim, 0)  


class RRTGraph: 
    """
    RRTGraph class manages the RRT graph structure and algorithm.

    Attributes:
    - start: Tuple representing the start position.
    - goal: Tuple representing the goal position.
    - map_dimensions: Tuple representing the dimensions of the map.
    - obsdim: Dimension of obstacles.
    - obsnum: Number of obstacles.
    - x: List storing x-coordinates of nodes.
    - y: List storing y-coordinates of nodes.
    - parent: List storing parent indices for each node.
    - goalFlag: Boolean indicating whether the goal has been reached.
    - goalstate: Index of the goal node.
    - path: List storing indices of nodes in the original path.
    - refined_path: List storing indices of nodes in the optimized path.
    - obstacles: List storing obstacle positions.

    Methods:
    - __init__: Constructor method to initialize the graph.
    - addObstacles: Method to convert obstacle positions to pygame.Rect objects.
    - add_node: Method to add a node to the graph.
    - remove_node: Method to remove a node from the graph.
    - add_edge: Method to add an edge between two nodes in the graph.
    - remove_edge: Method to remove an edge from the graph.
    - number_of_nodes: Method to get the number of nodes in the graph.
    - distance: Method to calculate Euclidean distance between two nodes in the graph.
    - sample_envir: Method to randomly sample a point within the map dimensions.
    - nearest: Method to find the nearest node to a given node in the graph.
    - isFree: Method to check if the last added node is in collision with any obstacles.
    - crossObstacle: Method to check if a line segment between two points crosses an obstacle.
    - connect: Method to attempt to connect two nodes with a straight line, avoiding obstacles.
    - step: Method to take a step towards a random sample, ensuring a maximum step distance and avoiding obstacles.
    - path_to_goal: Method to generate a path from the start to the goal using the parent relationships.
    - getPathCoords: Method to get the coordinates of nodes in the original path.
    - bias: Method to bias the exploration towards the goal.
    - expand: Method to expand the RRT graph by adding a new node.
    - optimize_path: Method to smooth the path by removing unnecessary waypoints.
    """
    def __init__(self,start,goal,mapDimensions,obsdim,obstacles,prohibited_zone):
        """
        Initialize RRTGraph with start and goal positions, map dimensions, obstacle dimensions, and the number of obstacles.

        ...
        """
        (x,y)=start
        self.start=start
        self.goal=goal
        self.goalFlag=False
        self.maph,self.mapw=mapDimensions
        self.x=[]
        self.y=[]
        self.parent=[]
        self.myRad = 38
        self.safety_distance=5
        #initialize the tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        #the obstacle
        self.obstacles=obstacles
        self.obsDim=obsdim 
        self.prohibited_zone = prohibited_zone
        #path
        self.goalstate = None 
        self.path=[]
        #smoothed path
        self.refined_path=[]

    def add_node(self, n, x, y) -> None:
        """
        Add a node to the graph with given coordinates.

        Parameters:
        - n (int): Index of the new node.
        - x (float): X-coordinate of the new node.
        - y (float): Y-coordinate of the new node.
        """        
        self.x.insert(n, x)
        if n != len(self.y):
            print(n, len(self.y))
        self.y.append(y)
        #self.y.insert(n, y)

    def remove_node(self,n): 
        """
        Remove a node from the graph.

        Parameters:
        - n (int): Index of the node to be removed.
        """        
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self,parent,child): 
        """
        Add an edge between two nodes in the graph.

        Parameters:
        - parent (int): Index of the parent node.
        - child (int): Index of the child node.
        """        
        self.parent.insert(child,parent)

    def remove_edge(self,n): 
        """
        Remove an edge from the graph.

        Parameters:
        - n (int): Index of the edge to be removed.
        """
        self.parent.pop(n)

    def number_of_nodes(self): 
        """
        Return the number of nodes in the graph.

        Returns:
        - count (int): Number of nodes in the graph.
        """        
        return len(self.x)
    
    def distance(self,n1,n2): 
        """
        Calculate Euclidean distance between two nodes in the graph.

        Parameters:
        - n1 (int): Index of the first node.
        - n2 (int): Index of the second node.

        Returns:
        - dist (float): Euclidean distance between the nodes.
        """        
        (x1,y1) = (self.x[n1],self.y[n1])
        (x2,y2) = (self.x[n2],self.y[n2])

        px=(float(x1)-float(x2))**2
        py=(float(y1)-float(y2))**2
        return ((px+py)**(0.5))
    
    def sample_envir(self):
        """
        Randomly sample a point within the map dimensions, excluding the prohibited zone.

        Args:
        - prohibited_zone (list): List containing the coordinates of the prohibited zone [x_min, x_max, y_min, y_max].

        Returns:
        - x (int): Randomly sampled x-coordinate.
        - y (int): Randomly sampled y-coordinate.
        """
        while True:
            x = int(random.uniform(0, self.mapw))
            y = int(random.uniform(0, self.maph))
            
            if not (self.prohibited_zone[0] <= x <= self.prohibited_zone[2] and self.prohibited_zone[1] <= y <= self.prohibited_zone[3]):
                return x, y

    
    def nearest(self,n): 
        """
        Find the nearest node to a given node in the graph.

        Parameters:
        - n (int): Index of the reference node.

        Returns:
        - nnear (int): Index of the nearest node.
        """
        dmin = self.distance(0,n)
        nnear = 0 
        for i in range(0,n):
            if self.distance(i,n)<dmin:
                dmin=self.distance(i,n)
                nnear=i
        return nnear    

    def isFree(self):
        """
        Check if the last added node is in collision with any obstacles.

        Returns:
        - free (bool): True if the node is collision-free, False otherwise.
        """    
        n = self.number_of_nodes()-1
        (x,y)=(self.x[n],self.y[n])
        obs=self.obstacles.copy()
        while len(obs) > 0:
            temp = obs.pop(0)
            distance = math.sqrt((x - temp[0])**2 + (y - temp[1])**2)
            limit = self.obsDim + self.safety_distance + self.myRad
            if distance < limit:
                self.remove_node(n)
                return False 
        return True     
    
    def crossObstacle(self, x1, x2, y1, y2):
        """
        Check if a line segment between two points crosses an obstacle.

        Parameters:
        - x1 (float): X-coordinate of the first point.
        - x2 (float): X-coordinate of the second point.
        - y1 (float): Y-coordinate of the first point.
        - y2 (float): Y-coordinate of the second point.
        - safety_distance (float): Safety distance to consider around obstacles.

        Returns:
        - crosses (bool): True if the line segment crosses an obstacle, False otherwise.
        """       
        obs = self.obstacles.copy()
        for obstacle in obs:
            for i in range(0, 101):
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)
                if math.hypot(x - obstacle[0], y - obstacle[1]) < (self.obsDim / 2 + self.safety_distance + self.myRad/2):
                    return True
        return False
    
    def connect(self, n1, n2):
        """
        Attempt to connect two nodes with a straight line, avoiding obstacles.

        Parameters:
        - n1 (int): Index of the first node.
        - n2 (int): Index of the second node.
        - safety_distance (float): Safety distance to consider around obstacles.

        Returns:
        - connected (bool): True if the nodes are successfully connected, False otherwise.
        """    
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])

        if self.crossObstacle(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True

    def step(self, nnear, nrand, dmax=50):
        """
        Take a step towards a random sample, ensuring a maximum step distance and avoiding obstacles.

        Parameters:
        - nnear (int): Index of the nearest node.
        - nrand (int): Index of the random sample node.
        - dmax (float): Maximum step distance.
        - safety_distance (float): Safety distance to consider around obstacles.
        """
        d = self.distance(nnear, nrand)

        if d > dmax:
            u = dmax / d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + (dmax - self.safety_distance) * math.cos(theta)), int(ynear + (dmax - self.safety_distance) * math.sin(theta)))

            self.remove_node(nrand)

            if abs(x - self.goal[0]) < dmax and abs(y - self.goal[1]) < dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand, x, y)
                

    def path_to_goal(self):
        """
        Generate a path from the start to the goal using the parent relationships.

        Returns:
        - success (bool): True if a path to the goal is successfully generated, False otherwise.
        """
        if self.goalFlag:
            self.path = []
            if 0 <= self.goalstate < len(self.parent):
                newpos = self.parent[self.goalstate]

                while newpos != 0 and newpos in self.parent:
                    self.path.append(newpos)
                    newpos = self.parent[newpos]

                if not self.path:
                    return False

                self.path.append(0)  
                self.path.reverse()
                return True

            else:
                return False
        else:
            return False


    def getPathCoords(self): 
        """
        Get the coordinates of nodes in the original path.

        Returns:
        - pathCoords (list): List of node coordinates in the original path.
        """
        pathCoords=[]
        for node in self.path:
            x,y=(self.x[node],self.y[node])
            pathCoords.append([x,y])
        x,y = self.goal[0],self.goal[1]    
        pathCoords.append([x,y])     
        return pathCoords            

    def bias(self,ngoal):
        """
        Bias the exploration towards the goal.

        Parameters:
        - ngoal (tuple): Coordinates of the goal node.

        Returns:
        - x (list): List of x-coordinates of nodes.
        - y (list): List of y-coordinates of nodes.
        - parent (list): List of parent indices for each node.
        """
        n=self.number_of_nodes()
        self.add_node(n,ngoal[0],ngoal[1])
        nnear=self.nearest(n)
        self.step(nnear,n)
        self.connect(nnear,n)
        return self.x,self.y,self.parent 

    def expand(self): 
        """
            Expand the RRT graph by adding a new node.

        Returns:
        - x (list): List of x-coordinates of nodes.
        - y (list): List of y-coordinates of nodes.
        - parent (list): List of parent indices for each node.
        """
        n = self.number_of_nodes()
        x,y=self.sample_envir()
        self.add_node(n,x,y)
        if self.isFree():
            xnearest=self.nearest(n)
            self.step(xnearest,n)
            self.connect(xnearest,n)
        return self.x,self.y,self.parent
        
    def optimize_path(self):
        """
        Smooth the path by removing unnecessary waypoints.

        Returns:
        - refined_path (list): List of indices representing the optimized (smoothed) RRT path.
        """
        self.getPathCoords()
        temporary_path = self.getPathCoords()
        print(len(temporary_path))
        self.refined_path.append(temporary_path[0])

        i = 0  
        n = len(temporary_path)  


        while i < n:
            last_valid_node = temporary_path[i+1]  


            j = n - 1
            while j > i:
                if not self.crossObstacle(temporary_path[i][0], temporary_path[j][0], temporary_path[i][1], temporary_path[j][1]):
                    last_valid_node = temporary_path[j]
                    self.refined_path.append(last_valid_node)
                    n = len(temporary_path)-(j-i)
                    break
                else:
                    j -= 1

            i = j
        if not (self.refined_path[-1] == self.goal):   
            x,y = self.goal[0],self.goal[1]    
            self.refined_path.append([x,y])  
        return self.refined_path