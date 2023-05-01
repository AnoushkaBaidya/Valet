""" ACKERMAN STEERING ROBOT VALET"""
from matplotlib.offsetbox import TextArea, DrawingArea, OffsetImage, AnnotationBbox
from matplotlib.patches import Rectangle
import matplotlib.image as mpimg
from queue import PriorityQueue
import matplotlib.pyplot as plt
import numpy as np
import cv2
import math


#Load Images for vehicles, obstacles and Ego Vehciles
veh1_path = '/Users/anoushka/Desktop/Motion Planning /HOMEWORKS/HW3/vehicle1.png'
veh2_path = '/Users/anoushka/Desktop/Motion Planning /HOMEWORKS/HW3/vehicle2.png'
ego_veh_path = '/Users/anoushka/Desktop/Motion Planning /HOMEWORKS/HW3/ego.png'
home_path = '/Users/anoushka/Desktop/Motion Planning /HOMEWORKS/HW3/prk.jpeg'

class Robot:
    def __init__(self) -> None:
        #Initiallise Ego Vehicle start and Stop position on the grid in between the two Vehicles  
        #Setting Vehicle obstacle dimensions
        self.veh_width = 30
        self.veh_height = 20
        #Positioning the vehicles in the grid
        self.l_vehicle = [30,10]
        self.r_vehicle=[130,10]
        #Initialise Ego Vehicle which we are controling
        self.ego_veh =[0,180,0] 
        self.ego_veh_start = [self.ego_veh[0] + 10, self.ego_veh[1] + self.veh_height/2,0]
        self.ego_veh_end = [self.l_vehicle[0] + self.veh_width + 15, 25 + self.veh_height/2,0]
        #Init robot wheel base and wheel radius parameters 
        self.robot_wRadius = 1
        self.robot_wBase = 28
        #Steering Angle of the robot 
        self.robot_steering_angle = 50
        #Initialising velocities
        self.vel = [-1,1]
        self.velocity = 1
        #Boundary of the robot
        self.robot_boundary = [[self.ego_veh[0],self.ego_veh[1],1],[self.ego_veh[0]+self.veh_width,self.ego_veh[1],1],[self.ego_veh[0]+self.veh_width,self.ego_veh[1]+self.veh_height,1],[self.ego_veh[0],self.ego_veh[1]+self.veh_height,1]]
        self.robot_bound = [[-1,29,29,-10],[-10,-10,10,10],[1,1,1,1]]
        self.i = 0
        #setting velocities for left and right wheel of the robot
        self.v_left = [5,2,1,0,-1,-2,-5]
        self.v_right = [5,2,1,0,-1,-2,-5]
        self.v = [5,3,0,-3,-5]
        
        

class World:
    def __init__(self,robot: Robot) -> None:
        self.robot= robot
        #Setting display values for the grid
        self.zoom_obs = 0.5
        self.zoom_veh1 = 0.07
        self.zoom_veh2 = 0.12
        #Setting buffer area around the obstacles 
        self.padding = 5
        #Initialise the position of obstacle and Vehicles 
        #obstacle x_pos, y_pos, width and height 
        self.obs = [70,90,60,60]
        #Position of the vehicles
        self.l_vehicle = [ 30,10]
        self.r_vehicle = [ 130,10]
        #Ego Vehicle position
        self.ego_veh = [ 0,180,0] 
        #Vehicle Dimensional Charecterestics
        self.vehicle_width = 30
        self.vehicle_height = 20

        #Obstacle and Vehicle Vertices
        padding = self.padding
        obs_x, obs_y, obs_size,obs_size = self.obs
        obstacle_vertices = []
        for dx, dy in [(-padding, -padding), (obs_size+padding, -padding),
                       (obs_size+padding, obs_size+padding), (-padding, obs_size+padding)]:
            vertex = [obs_x + dx, obs_y + dy]
            obstacle_vertices.append(vertex)

        self.obstacle = obstacle_vertices
        self.i = 0
        vehicle_width, vehicle_height = self.vehicle_width, self.vehicle_height

        # Define the vertices of the left vehicle
        lx, ly = self.l_vehicle
        vehicle1_vertices = []
        for dx, dy in [(-padding, -padding), (vehicle_width+padding, -padding),
                       (vehicle_width+padding, vehicle_height+padding), (-padding, vehicle_height+padding)]:
            vertex = [lx + dx, ly + dy]
            vehicle1_vertices.append(vertex)
        self.vehicle1 = vehicle1_vertices

        # Define the vertices of the right vehicle
        rx, ry = self.r_vehicle
        vehicle2_vertices = []
        for dx, dy in [(-padding, -padding), (vehicle_width+padding, -padding),
                       (vehicle_width+padding, vehicle_height+padding), (-padding, vehicle_height+padding)]:
            vertex = [rx + dx, ry + dy]
            vehicle2_vertices.append(vertex)
        self.vehicle2 = vehicle2_vertices

    def environment(self,x,y,theta):
        fig = plt.figure("Valet")
        ax = fig.add_subplot(111)
        #Adding the Vehicles and Obstacles in the environment display 
        obs_p = [self.obs[0]+61.5, self.obs[1]+26.5]
        v1_p = [50, 0.8]
        v2_p = [155,0.8]  
                    
        ego_veh = mpimg.imread(ego_veh_path)
        #Setting Ego vehicle start position 
        #plotting our Ego Vehicle updated PPositions
        imagebox_4 = self.rotate_bound(ego_veh, -theta)
        imagebox_4 = OffsetImage(imagebox_4, zoom=0.17)

        
        if x != self.robot.ego_veh_end[0] and y !=self.robot.ego_veh_end[1]:
            x = x+30
            y = y-30

        ab_4 = AnnotationBbox(imagebox_4, (x,y),bboxprops =dict(edgecolor='white')) # updating vehicle
        ax.add_artist(ab_4)



        #Setting the vehicles and Obstacles in the plot
        home =  mpimg.imread(home_path)
        imagebox = OffsetImage(home, zoom=self.zoom_obs)
        ab = AnnotationBbox(imagebox,(obs_p[0],obs_p[1]), bboxprops=dict(edgecolor='white'))
        ax.add_artist(ab)
        #ax.set_xlim(obs_p[0]-home.size[0]*self.zoom_obs, obs_p[0]+home.size[0]+home.size[0]*self.zoom_obs)
        #ax.set_ylim(obs_p[1]-home.size[1]*self.zoom_obs, obs_p[1]+home.size[1]+home.size[1]*self.zoom_obs)

        #Setting the vehicles and Obstacles in the plot
        veh1 = mpimg.imread(veh1_path)
        imagebox = OffsetImage(veh1, zoom=self.zoom_veh1)
        ab = AnnotationBbox(imagebox,(v1_p[0],v1_p[1]), bboxprops=dict(edgecolor='white'))
        ax.add_artist(ab)
        #ax.set_xlim(v1_p[0]-veh1.size[0]*self.zoom_veh1, v1_p[0]+veh1.size[0]+veh1.size[0]*self.zoom_veh1)
        #ax.set_ylim(v1_p[1]-veh1.size[1]*self.zoom_veh1, v1_p[1]+veh1.size[1]+veh1.size[1]*self.zoom_veh1)
        
        veh2 = mpimg.imread(veh2_path)
        imagebox = OffsetImage(veh2, zoom=self.zoom_veh2)
        ab = AnnotationBbox(imagebox,(v2_p[0],v2_p[1]), bboxprops=dict(edgecolor='white'))
        ax.add_artist(ab)
        #ax.set_xlim(v2_p[0]-veh2.size[0]*self.zoom_veh, v2_p[0]+veh2.size[0]+veh2.size[0]*self.zoom_veh)
        #ax.set_ylim(v2_p[1]-veh2.size[1]*self.zoom_veh, v2_p[1]+veh2.size[1]+veh2.size[1]*self.zoom_veh)

        #Setting the limits on the plot grid
        ax.axis('off')
        plt.xlim([0,200])
        plt.ylim([-20,200])
        self.i = self.i + 1

        return 

    def rotate_bound(self,image, angle):
        
        # Get the dimensions of the image and determine the center
        height, width = image.shape[:2]
        center = (width // 2, height // 2)

        # Get the rotation matrix and calculate the sine and cosine
        rotation_matrix = cv2.getRotationMatrix2D(center, -angle, 1.0)
        cos = np.abs(rotation_matrix[0, 0])
        sin = np.abs(rotation_matrix[0, 1])

        # Calculate the new bounding dimensions of the image
        new_width = int((height * sin) + (width * cos))
        new_height = int((height * cos) + (width * sin))

        # Adjust the rotation matrix to take into account translation
        rotation_matrix[0, 2] += (new_width / 2) - center[0]
        rotation_matrix[1, 2] += (new_height / 2) - center[1]

        # Perform the rotation and return the image
        return cv2.warpAffine(image, rotation_matrix, (new_width, new_height), borderValue=(255, 255, 255))


    def bound(self,x,y,theta):
        theta_new = (theta - self.robot.ego_veh_start[2])*(math.pi/180)
        trans_matrix = [[math.cos(theta_new),-math.sin(theta_new),x],[math.sin(theta_new),math.cos(theta_new),y]]
        matrix = np.dot(trans_matrix,self.robot.robot_bound)
        updated_boundary = []
        for i in range(len(matrix[0])):
            updated_boundary.append([matrix[0][i], matrix[1][i]])
        return updated_boundary
    

class Controller():
    def __init__(self,robot = Robot , world= World ) -> None:
        self.robot = robot
        self.world = world

    #Controller ensures it gives collision free path 
    def intersection_check(self,polygonA, polygonB):
        polygons = [polygonA, polygonB]
        for polygon in polygons:
            for i in range(len(polygon)):
                vertice_1 = i
                vertice_2 = (i + 1) % len(polygon)
                p1 = polygon[vertice_1]
                p2 = polygon[vertice_2]
                normal = {'x': p2[1] - p1[1], 'y': p1[0] - p2[0]}
                minA, maxA, minB, maxB = None, None, None, None
                for k in range(len(polygonA)):
                    projected = normal['x'] * polygonA[k][0] + normal['y'] * polygonA[k][1]
                    minA = projected if minA is None else min(minA, projected)
                    maxA = projected if maxA is None else max(maxA, projected)
                for k in range(len(polygonB)):
                    projected = normal['x'] * polygonB[k][0] + normal['y'] * polygonB[k][1]
                    minB = projected if minB is None else min(minB, projected)
                    maxB = projected if maxB is None else max(maxB, projected)
                if maxA < minB or maxB < minA:
                    return False
        return True
        
 
class Planner():
    def __init__(self,robot = Robot , world= World, controller= Controller ) -> None:
        self.robot = robot
        self.world = world
        self.controller= controller

    def check_visited(self,curr_node,visited):
        return curr_node in [(x, y, theta) for x, y, theta in visited]

    #g(x) function of A_star
    def dist_cost(self,x1,y1,x2,y2):
      return math.sqrt((pow(x1-x2,2)+pow(y1-y2,2)))

    def valid_point(self,x,y,theta):
        if x < 1 or y < self.world.vehicle_height or x > 200 - self.world.vehicle_width or y > 200 - self.world.vehicle_height/2.0:
            return False

        # Check for collisions with obstacles and other vehicles
        boundary = self.world.bound(x, y, theta)
        for polygon in [self.world.obstacle, self.world.vehicle1, self.world.vehicle2]:
            if self.controller.intersection_check(boundary, polygon):
                return False
        return True
    
    def get_neighbours(self, x, y, theta):
        neighbours = []
        for i in range(-self.robot.robot_steering_angle,self.robot.robot_steering_angle+1,5):
            x_dot = self.robot.velocity * math.cos(theta*(math.pi/180))
            y_dot = self.robot.velocity * math.sin(theta*(math.pi/180))
            theta_dot = (self.robot.velocity*math.tan(i*(math.pi/180))/self.robot.robot_wBase)*(180/math.pi)
            if(self.valid_point(x+x_dot,y+y_dot,theta+theta_dot)):
                neighbours.append([round(x+x_dot,2),round(y+y_dot,2),(round(theta+theta_dot,2))%360,1,i])
            if(self.valid_point(x-x_dot,y-y_dot,theta-theta_dot)):
                neighbours.append([round(x-x_dot,2),round(y-y_dot,2),(round(theta-theta_dot,2))%360,-1,i])
        return neighbours

    def path_available(self, x, y):   # Define boundary line segment
        p1 = [x, y]
        p2 = [self.robot.ego_veh_end[0], self.robot.ego_veh_end[1]]
        p3 = [self.robot.ego_veh_end[0]+1, self.robot.ego_veh_end[1]]
        p4 = [x+1, y]
        boundary_line = [p1, p2, p3, p4]
    
        # Check for collisions with obstacles and other vehicles
        obstacles = [self.world.obstacle, self.world.vehicle1,self.world.vehicle2]
        for obstacle in obstacles:
            if self.controller.intersection_check(boundary_line, obstacle):
                return False
        return True

    def is_at_goal(self,x,y):
        if (x > self.robot.ego_veh_end[0]-1 and y > self.robot.ego_veh_end[1]-1 and x <self.robot.ego_veh_end[0]+1 and y <self.robot.ego_veh_end[1]+1):
            return True

    #h(x)
    def rob_move(self, x, y, theta):
        # Calculate Euclidean distance between current position and goal position
        diff_heading = 0
        theta = (theta +360)%360
        dx = self.robot.ego_veh_end[0] - x
        dy = self.robot.ego_veh_end[1] - y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        dx2 = (self.robot.ego_veh_end[0] + self.world.vehicle_width) - (x + self.world.vehicle_width * math.cos(theta * (math.pi / 180)))
        dy2 = (self.robot.ego_veh_end[1] + self.world.vehicle_height) - (y + self.world.vehicle_height * math.sin(theta * (math.pi / 180)))
        distance += math.sqrt(dx2 ** 2 + dy2 ** 2)
        # Calculate distance to reach goal position along a straight line
        #straight_distance = math.sqrt(((x + self.world.vehicle_width * math.cos(theta * math.pi / 180) - self.robot.ego_veh_end[0] - self.world.vehicle_width) ** 2 + (y + self.world.vehicle_height * math.sin(theta * math.pi / 180) - self.robot.ego_veh_end[1] - self.world.vehicle_height) ** 2))

        # Check for obstacles in the way of the straight line path
        if self.path_available(x, y) and not self.is_at_goal(x,y): 
            dx = x - self.robot.ego_veh_end[0]
            dy = y - self.robot.ego_veh_end[1]
            goal_heading = (math.atan2(dy, dx))
            diff_heading =  abs((360 + goal_heading*(180/math.pi))%360 - theta+180)

        heuristic = distance + diff_heading

        return heuristic

    def AStar_path_planner(self):
        visited_nodes = []
        #Initialise list and dictionaries to keep track of g_score and f_score 
        g_val = {}
        f_val=[]
        start = [10, 190.0, 0]  #Starting Node -> x,y and heading(theta)
        print("Starting Node: {f}".format(f=start))
        f = 0
        g = 0
        #A priority queue is initialized and the starting node is added to it with f and g values initialized to 0 and an empty path.
        path = start
        prior_queue = PriorityQueue()
        prior_queue.put((f,g,start,[path]))

        while not prior_queue.empty():

            #Inside the loop, the node with the lowest f value is retrieved from the priority queue and its coordinates, g_score, f_score and path are extracted.
            # get the next item from the priority queue
            f_score, g_score, curr, path = prior_queue.get()

            # check if the current node has already been visited
            if not self.check_visited((round(curr[0]),round(curr[1]),round(curr[2])), visited_nodes):
                visited_nodes.append((round(curr[0]),round(curr[1]),round(curr[2])))

                # check if the current node is the goal
                ego_x, ego_y, ego_theta = self.robot.ego_veh_end
                if (ego_x - 5 <= curr[0] <= ego_x + 5) and (ego_y - 5 <= curr[1] <= ego_y + 5) and (ego_theta - 15 <= curr[2] <= ego_theta + 15):
                    return path

                # get the neighbors of the current node
                neighbours = self.get_neighbours(curr[0], curr[1], curr[2])

                for neighbour in neighbours:
                    # calculate the distance cost from the current node to the neighbor
                    #dist_cost = math.sqrt((pow(curr[0]-curr[1],2)+pow(neighbour[0]-neighbour[1],2)))
                    #The g_score and f_score of each neighbor is calculated and added to the priority queue.
                    # calculate the g value for the neighbor
                    g_val = g_score + (0.1*self.dist_cost(curr[0],curr[1],neighbour[0],neighbour[1]))

                    # calculate the f value for the neighbor
                    f_score = g_val + (2*self.rob_move(neighbour[0],neighbour[1],neighbour[2]))

                    # add the neighbor to the priority queue
                    prior_queue.put((f_score, g_val, neighbour, path + [neighbour]))

        return path


if __name__ == "__main__":
    Ack_rob = Robot()
    world = World(Ack_rob)
    Ack_controller = Controller(Ack_rob,world)
    Ack_planner = Planner(Ack_rob,world,Ack_controller) 
    path_values_calculated = Ack_planner.AStar_path_planner()
    print("trajectory",path_values_calculated)
    print("TRAJECTORY FOUND. I WILL PARK THE CAR DONT WORRY !")

    for path_val in path_values_calculated:
        world.environment(path_val[0],path_val[1],path_val[2])
        plt.pause(0.00001)

#Plotting the Path taken by the Vehcile 
plt.figure("Position Plot of ackermann_steering")
array = []
array_y = []
for point in path_values_calculated:
    array.append([point[0]])
    array_y.append([point[1]])

plt.plot(array,array_y,'g--')
plt.show()  
