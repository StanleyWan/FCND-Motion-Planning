import sys
#!{sys.executable} -m pip install -I networkx==2.1
import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx

nx.__version__

import argparse
import time
import msgpack
from bresenham import bresenham
import matplotlib.pyplot as plt
#%matplotlib inline
from enum import Enum, auto

import numpy as np
from voronoi_util import create_grid_and_edges
from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

from queue import PriorityQueue
import numpy.linalg as LA

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, waypoints=[]):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = waypoints
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):  ##
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()
        print("arming transition Done")

    def takeoff_transition(self):  ##
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):  ##
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):  ##
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):   ## NEW
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        #print ("data is ",data)
        self.connection._master.write(data)


    def plan_path(self):    ## NEW
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE
        if len(self.waypoints) > 0:
            print("send waypoint() !!!")
            self.send_waypoints()
            time.sleep(1)
        else:
            # TODO: read lat0, lon0 from colliders into floating point values
            # Stanley: read the first line and break down into tokens
            F1 = open('colliders.csv', 'r')
            S1 = F1.readline(-1)            
            S2, S3 = S1.split(", ")
            _, S4 = S2.split()
            _, S5 = S3.split()
            lat0 = float(S4)
            lon0 = float(S5)
            print (lat0)
            print (lon0)           
            
            # TODO: set home position to (lon0, lat0, 0)
            self.set_home_position(lon0, lat0, 0)
            print ("The global home is ", self.global_home)  
        
            # TODO: retrieve current global position
            global_pos_current = [self._longitude, self._latitude, self._altitude] #new
            print("the longitude, latitude and altitude are : ", self._longitude, self._latitude, self._altitude)
            print ("The global position is ", self.global_position)
 
            # TODO: convert to current local position using global_to_local()
            #self.local_position = global_to_local(self.global_position, self.global_home)
            local_north, local_east, local_down = global_to_local(global_pos_current, self.global_home)
            #local_north, local_east, local_down = global_to_local(self.global_position, self.global_home)
            print ("local_north is", local_north)
            print ("local_east is", local_east)
            print ("local_down is", local_down)
            print ("The local position is ", self.local_position)

        
            print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
            # Read in obstacle map
            data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
            
            print ("CSV File loaded !!!")
        
            # Define a grid for a particular altitude and safety margin around obstacles
            grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
            #grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
            plt.rcParams["figure.figsize"] = [12,12]
            fig = plt.figure()     
            plt.imshow(grid, origin='lower')
            plt.title('A* Motion Path Graph')
            plt.xlabel('EAST')           
            plt.ylabel('NORTH')          
            #plt.show()

            print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
            # Define starting point on the grid (this is just grid center)
            #grid_start = (-north_offset, -east_offset)
            # TODO: convert start position to current position rather than map center (stanley: it should be offset point, not center)
            grid_start = (int(np.ceil(self.local_position[0] - north_offset)), int(np.ceil((self.local_position[1] - east_offset))))
            #grid_start = (int(np.ceil(400)), int(np.ceil(445)))
            
            # Set goal as some arbitrary position on the grid
            #grid_goal = (int(np.ceil(-north_offset + 300)), int(np.ceil(-east_offset + 100)))
            #grid_goal = (int(np.ceil(-north_offset + 10)), int(np.ceil(-east_offset + 10)))
            """
            #in case, the command line argument input is longiture, latitude and altitude
            global_goal = [float(args.goal_lon), float(args.goal_lat), 0.0]
            print ("global_goal ", global_goal[0], global_goal[1])            
            local_goal = global_to_local(global_goal, self.global_home)
            print("the local goal is ", local_goal[0], local_goal[1])
            #grid_goal = (int(np.ceil(800)), int(np.ceil(180)))
            grid_goal = (int(np.ceil(-north_offset + local_goal[0])), int(np.ceil(-east_offset + local_goal[1])))
            print("the grid_goal is ", grid_goal[0], grid_goal[1])
            """
            
            grid_goal = (int(np.ceil(args.grid_goal_north)), int(np.ceil(args.grid_goal_east)))
            
            if  (grid[(grid_goal)] == 1):
                print ("The goal is inside the obstacle")
                sys.exit()
            else :
                print ("The goal is outside the obstacle")
        
            

            # TODO: adapt to set goal as latitude / longitude position and convert
           
            #*******************  Run A Star Search  ************************
        
            # Run A* to find a path from start to goal
            # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
            # or move to a different search space such as a graph (not done here)
            print('Local Start and Goal: ', grid_start, grid_goal)

            self.disarming_transition()
            self.manual_transition()

            path, _ = a_star(grid, heuristic, grid_start, grid_goal)
            if (path==[]):
                sys.exit()
            # TODO: prune path to minimize number of waypoints
            # TODO (if you're feeling ambitious): Try a different approach altogether!
            pruned_path = bres_prune(grid, path) 
            
            p = np.array(path)
            plt.plot(p[:,1], p[:,0],'g', label="A Star Motion Path")
            plt.legend(loc='best')  
            fig.savefig('A Star Motion Path Graph.png')
            
            pp = np.array(pruned_path)
            plt.plot(pp[:,1], pp[:,0],'r', label="Pruned A Star Motion Path")
            plt.legend(loc='best')
            plt.title('A* Pruned Motion Path Graph')
            #plt.show()            
            fig.savefig('A Star Pruned Motion Path Graph.png')
            print ("A Star Motion Path Graph Saved")

            ##**********  Create Voronoi Map **********
            
            plt.rcParams["figure.figsize"] = [12,12]
            fig = plt.figure()     
            plt.imshow(grid, origin='lower')
            plt.xlabel('EAST')           
            plt.ylabel('NORTH')             
            grid, edges = create_grid_and_edges(data, 5, 3)

            plt.imshow(grid, origin='lower', cmap='Greys') 

            # Stepping through each edge
            for e in edges:
                p1 = e[0]
                p2 = e[1]
                plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-', label='voronoi Map')
            plt.legend(loc='best')

            plt.xlabel('EAST')
            plt.ylabel('NORTH')
            plt.title('Voronoi Map')
            fig.savefig('Voronoi Map.png')
            print ("voronoi Map Saved")
            
            ## ***********  Do a Search on voronoi Map  *************
            
            # TODO: create the graph with the weight of the edges
            # set to the Euclidean distance between the points
            G = nx.Graph()
            for e in edges:
                p1 = e[0]
                p2 = e[1]
                dist = LA.norm(np.array(p2) - np.array(p1))
                G.add_edge(p1, p2, weight=dist)
                
                
            start_ne = grid_start
            goal_ne = grid_goal
            start_ne_g = closest_point_voronoi(G, start_ne)
            goal_ne_g = closest_point_voronoi(G, goal_ne)
            print("start_ne_g =", start_ne_g)
            print("goal_ne_g = ", goal_ne_g)
            
            path, cost = a_star_voronoi(G, heuristic_voronoi, start_ne_g, goal_ne_g)
            if (path==[]):
                sys.exit()
            
            print(len(path))
                                    
            pruned_path_voronoi = bres_prune(grid, path) 
            
            # equivalent to
            # plt.imshow(np.flip(grid, 0))
            plt.imshow(grid, origin='lower', cmap='Greys') 

            #  *** Draw the voronoi graph ***
            """
            for e in edges:
                p1 = e[0]
                p2 = e[1]
                plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
            

            plt.plot([start_ne[1], start_ne_g[1]], [start_ne[0], start_ne_g[0]], 'r-')
            """
            for i in range(len(path)-1):
                p1 = path[i]
                p2 = path[i+1]
                plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')
            plt.plot([goal_ne[1], goal_ne_g[1]], [goal_ne[0], goal_ne_g[0]], 'r-',label="Voronoi Motion Path")
            plt.xlabel('EAST', fontsize=20)
            plt.ylabel('NORTH', fontsize=20)
            plt.legend(loc='best')
            plt.title('Voronoi Motion Path Graph')
            fig.savefig('Voronoi Motion Path.png')
            
            for i in range(len(pruned_path_voronoi)-1):
                p1 = pruned_path_voronoi[i]
                p2 = pruned_path_voronoi[i+1]
                plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'g-')
            plt.plot([goal_ne[1], goal_ne_g[1]], [goal_ne[0], goal_ne_g[0]], 'g-', label="Pruned voronoi Motion Path")
            

            plt.plot(start_ne[1], start_ne[0], 'gx')
            plt.plot(goal_ne[1], goal_ne[0], 'gx')

            plt.xlabel('EAST', fontsize=20)
            plt.ylabel('NORTH', fontsize=20)
            plt.legend(loc='best')
            #plt.show()
            plt.title('Voronoi Pruned Motion Path Graph')
            fig.savefig('Voronoi Pruned Motion Path.png', bbox_inches='tight')
            print ("Graphic Done")

            
            # Convert path to waypoints
            waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in pruned_path_voronoi]
            #waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in pruned_path]
            #waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in pruned_path]
            print ("waypoints is ", waypoints)
            # Set self.waypoints        
        
            self.waypoints = waypoints

            print("starting connection again")
            conn = MavlinkConnection('tcp:{0}:{1}'.format('127.0.0.1', 5760), timeout=600)
            drone = MotionPlanning(conn, waypoints=self.waypoints)
            time.sleep(1)
            drone.start()
            
            # TODO: send waypoints to sim (this is just for visualization of waypoints)
            #self.send_waypoints()
            
    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()

# Define a simple function to add a z coordinate of 1
def point(p):
    return np.array([p[0], p[1], 1.])

def collinearity_float(p1, p2, p3, epsilon=1e-6): 
    collinear = False
    # Create the matrix out of three points
    # Add points as rows in a matrix
    mat = np.vstack((point(p1), point(p2), point(p3)))
    # Calculate the determinant of the matrix. 
    det = np.linalg.det(mat)
    # Set collinear to True if the determinant is less than epsilon
    #print ("det is ", det)
    if det < epsilon:
        collinear = True
        
    return collinear

def collinearity_int(p1, p2, p3): 
    collinear = False
    # Calculate the determinant of the matrix using integer arithmetic 
    det = p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])
    # Set collinear to True if the determinant is equal to zero
    #print ("det is ", det)
    if det == 0:
        collinear = True

    return collinear

""" Note prune_waypoints cannot detech the obstacles,  so we need to use bres_prune"""
def prune_waypoints(path):
    #pruned_path = path
    pruned_path = [p for p in path]
    #print("pruned path is ", pruned_path)  
    i = 0
    while len(pruned_path)-i >=3:
        p1 = pruned_path[i+0]
        p2 = pruned_path[i+1]
        p3 = pruned_path[i+2]
        if (collinearity_float(p1, p2, p3) == True):
            pruned_path.remove(p2)
        else:
            i += 1
    #print("pruned path is ", pruned_path)
    return pruned_path



def heuristic_voronoi(n1, n2):
    return LA.norm(np.array(n2) - np.array(n1))

def a_star_voronoi(graph, h, start, goal):
    """Modified A* to work with NetworkX graphs."""
    
    print ("start and goal = ", start, goal)
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)
    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        #print ("current_node = ", current_node)
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            #print ("neighbor = ", graph[current_node] )
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                branch_cost = current_cost + cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))
                    
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
        path = []
    return path[::-1], path_cost

def closest_point_voronoi(graph, current_point):
    """
    Compute the closest point in the `graph`
    to the `current_point`.
    """
    closest_point = None
    dist = 100000
    for p in graph.nodes:
        d = LA.norm(np.array(p) - np.array(current_point))
        if d < dist:
            closest_point = p
            dist = d
    return closest_point

def bres_prune(grid, path):    
    
    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1 = pruned_path[i]
        p2 = pruned_path[i + 1]
        p3 = pruned_path[i + 2]
        
        if  all((grid[cell] == 0) for cell in bresenham(int(p1[0]), int(p1[1]), int(p3[0]), int(p3[1]))):
            pruned_path.remove(p2)
        else:
            i += 1
            
    return pruned_path

"""
def bres_prune(grid, path):
    
    #Use the Bresenham module to trim uneeded waypoints from path
    
    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1 = pruned_path[i]
        p2 = pruned_path[i + 1]
        p3 = pruned_path[i + 2]
        # if the line between p1 and p2 doesn't hit an obstacle
        # remove the 2nd point.
        # The 3rd point now becomes the 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if  all((grid[pp] == 0) for pp in bresenham(int(p1[0]), int(p1[1]), int(p3[0]), int(p3[1]))):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is checked on every iteration.
            pruned_path.remove(p2)

        else:
            i += 1
    return pruned_path
"""

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    #parser.add_argument('--goal_lon', type=float, default=-122.39645, help="Goal Longitude")
    #parser.add_argument('--goal_lat', type=float, default=37.79148, help="Goal Latitude")
    parser.add_argument('--grid_goal_north', type=int, default=800, help="Goal Latitude")
    parser.add_argument('--grid_goal_east', type=int, default=100, help="Goal Latitude")
    
    #args = parser.parse_args()
    args, unknown = parser.parse_known_args()
    
    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=600)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start() 