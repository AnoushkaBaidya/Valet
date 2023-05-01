""" ACKERMAN STEERING TRUCK WITH A TRAILER """

from matplotlib.offsetbox import TextArea, DrawingArea, OffsetImage, AnnotationBbox
from matplotlib.patches import Rectangle
import matplotlib.patches as mpatches
from matplotlib import get_backend
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
from queue import PriorityQueue
import numpy as np
import math
import cv2

#Load Images for vehicles, obstacles and Ego Vehciles
veh1_path = '/Users/anoushka/Desktop/Motion Planning /HOMEWORKS/HW3/vehicle1.png'
veh2_path = '/Users/anoushka/Desktop/Motion Planning /HOMEWORKS/HW3/vehicle2.png'
ego_veh_trailer_path = '/Users/anoushka/Desktop/Motion Planning /HOMEWORKS/HW3/ackermann_trailer.png'
home_path = '/Users/anoushka/Desktop/Motion Planning /HOMEWORKS/HW3/prk.jpeg'
ego_veh_path = '/Users/anoushka/Desktop/Motion Planning /HOMEWORKS/HW3/trailer.png'


class Robot:
    def __init__(self) -> None:
        #Initiallise Ego Vehicle start and Stop position on the grid in between the two Vehicles  
        #Setting Vehicle obstacle dimensions
        self.ego_veh = [49,180]
        self.trailer = [self.ego_veh[0]-28,self.ego_veh[1]]
        self.veh_width = 22
        self.veh_height = 17.5
        self.padding = 2
        #Positioning the vehicles in the grid
        self.obs = [70,90,50,50]
        self.l_veh = [30,10]    
        #Setting starting and goal positions of the ego vehicle truck with trailer
        self.ego_veh_start = [self.ego_veh[0], self.ego_veh[1] + self.veh_height/2,0]
        self.trailer_start = [self.trailer[0],self.trailer[1] +self.veh_height/2,0]
        self.ego_veh_goal = [self.l_veh[0] + 2*self.veh_width + 30, 10 + self.veh_height/2,180]
        self.robot_wRadius = 1
        self.robot_wBase = 15
        self.robot_steering_angle = [-40 ,0 ,40]
        self.conn_l = 25
        #setting velocities for left and right wheel of the robot
        self.velocity = [-1,1]
        #Boundary of the truck and trailer
        self.robot_boundary = [[self.ego_veh[0],self.ego_veh[1],1],[self.ego_veh[0]+self.veh_width,self.ego_veh[1],1],[self.ego_veh[0]+self.veh_width,self.ego_veh[1]+self.veh_height,1],[self.ego_veh[0],self.ego_veh[1]+self.veh_height,1]]
        self.robot_bound = [[-1,21,21,-1],[-7.5,-7.5,7.5,7.5],[1,1,1,1]]
        self.trailer_bound = [[-1,11,11,-1],[-7.5,-7.5,7.5,7.5],[1,1,1,1]]


class World:
    def __init__(self,robot: Robot) -> None:
        self.robot = robot
        #Initialise the position of obstacle and Vehicles 
        #obstacle x_pos, y_pos, width and height
        self.obs = [70,90,50,50] 
        #Position of the vehicles
        self.l_veh = [30,10]
        self.padding = 2
        #Truck wheel charectersestics 
        self.robot_wRadius = 1
        self.robot_wBase = 15
        #Defining Velocity of the truck
        self.velocity = [-1,1]
        self.veh_height = 17.5
        self.park_point = False
        #vehicle1 charecterserstics 
        self.zoom_veh1 = 0.08
        self.veh_1 = [50,25]

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
        vehicle_width, vehicle_height = self.robot.veh_width, self.robot.veh_height

        #Define the vertices of the left vehicle
        lx, ly = self.l_veh
        vehicle1_vertices = []
        for dx, dy in [(-padding, -padding), (vehicle_width+padding, -padding),
                       (vehicle_width+padding, vehicle_height+padding), (-padding, vehicle_height+padding)]:
            vertex = [lx + dx, ly + dy]
            vehicle1_vertices.append(vertex)
        self.vehicle1 = vehicle1_vertices

    def environment(self,x,y,theta,x_t,y_t,theta_t):
        fig = plt.figure("Valet")
        ax = fig.add_subplot(111)
        #Adding the Vehicles and Obstacles in the environment display 
      
        #Setting the vehicles and Obstacles in the plot
        veh1 = mpimg.imread(veh1_path)
        imagebox = OffsetImage(veh1, zoom=self.zoom_veh1)
        ab = AnnotationBbox(imagebox, (self.veh_1[0],self.veh_1[1]),bboxprops =dict(edgecolor='white'))
        ax.add_artist(ab)
        
        """ TRAILER EGO VEHICLE """
        #Setting Ego vehicle start position 
        #plotting our Ego Vehicle updated Positions
        ego_t = mpimg.imread(ego_veh_trailer_path)
        imagebox_4 = self.rotate_bound(ego_t, -theta_t)
        imagebox_4 = OffsetImage(imagebox_4, zoom=0.38)
        ab_4 = AnnotationBbox(imagebox_4, (x_t,y_t+10),bboxprops =dict(edgecolor='white')) # updating vehicle trailer position throughout 
        ax.add_artist(ab_4)

        ego_veh = mpimg.imread(ego_veh_path)
        imagebox_5 = self.rotate_bound(ego_veh, -theta)
        imagebox_5 = OffsetImage(imagebox_5, zoom=0.44)
        ab_4 = AnnotationBbox(imagebox_5, (x,y-5),bboxprops =dict(edgecolor='white')) # updating vehicle position throughout
        ax.add_artist(ab_4)

        #Adding the obstacle in the grid image
        home = mpimg.imread(home_path)
        imagebox = OffsetImage(home, zoom=0.38)
        ab = AnnotationBbox(imagebox, (self.obs[0]+5, self.obs[1]+15), bboxprops=dict(edgecolor='white'))
        ax.add_artist(ab)
        ax.axis('off')
        
        plt.xlim([0,200])
        plt.ylim([0,200])
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


    def bound(self,x,y,theta,vehicle_type ='car'):
        theta_new = (theta - self.robot.ego_veh_start[2])*(math.pi/180)
        trans_matrix = [[math.cos(theta_new),-math.sin(theta_new),x],[math.sin(theta_new),math.cos(theta_new),y]]
        if vehicle_type == 'car':
            matrix = np.dot(trans_matrix,self.robot.robot_bound)
        else:
            matrix = np.dot(trans_matrix,self.robot.trailer_bound)

        updated_boundary = []
        for i in range(len(matrix[0])):
            updated_boundary.append([matrix[0][i], matrix[1][i]])
        return updated_boundary
    
    def get_boundary(self,x,y,theta,vehicle_type ='car'):
        theta_new = (theta - self.robot.ego_veh_start[2])*(math.pi/180)
        trans_matrix = [[math.cos(theta_new),-math.sin(theta_new),x],[math.sin(theta_new),math.cos(theta_new),y]]
        if vehicle_type == 'car':
            matrix = np.dot(trans_matrix,self.robot.robot_bound)
        else:
            matrix = np.dot(trans_matrix,self.robot.trailer_bound)

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

    
    def get_neighbours(self,x,y,theta,xt,yt,theta_t):
        neighbours = []
        for angle in self.robot.robot_steering_angle:
          for vel in self.world.velocity:
            x_dot = x + vel * math.cos(theta*(math.pi/180))
            y_dot = y+ vel * math.sin(theta*(math.pi/180))
            theta_dot = theta + (vel*math.tan(angle*(math.pi/180))/self.world.robot_wBase)*(180/math.pi)
            xt_dot = xt + vel * math.cos(theta_t*(math.pi/180))
            yt_dot = yt + vel * math.sin(theta_t*(math.pi/180))
            theta_t_dot = theta_t+ (vel * math.sin((theta-theta_t)*(math.pi/180))/self.robot.conn_l)*(180/math.pi)
            if(self.is_valid_point(x_dot,y_dot,theta_dot,xt_dot,yt_dot, theta_t_dot)):
                neighbours.append([round(x_dot,2),round(y_dot,2),(round(theta_dot,2))%360,round(xt_dot,2),round(yt_dot,2),(round(theta_t_dot,2)+360)%360,vel,angle])
    
        return neighbours
    
    def check_visited(self,curr_node,visited):
        return curr_node in visited
    
    def is_valid_point(self, x, y, theta, xt, yt, theta_t):
        # Check if the point is within the boundaries of the world
        if not (1 <= x <= 200 - self.robot.veh_width and
                self.world.veh_height <= y <= 200 - self.world.veh_height/2.0):
            return False

        # Check for collisions with obstacles and other vehicles
        boundary = self.world.get_boundary(x, y, theta)
        boundary_t = self.world.get_boundary(xt, yt, theta_t, "trailer")
        for polygon in [self.world.obstacle, self.world.vehicle1]:
            if self.controller.intersection_check(boundary, polygon) or \
               self.controller.intersection_check(boundary_t, polygon):
                return False

        return True
    
    def dist_cost(self,x1,y1,x2,y2):
        return math.sqrt((pow(x1-x2,2)+pow(y1-y2,2)))
    
    def path_available(self,x,y,xt,yt):
        boundary_line = [[x,y],[self.robot.ego_veh_goal[0],self.robot.ego_veh_goal[1]],[self.robot.ego_veh_goal[0]+1,self.robot.ego_veh_goal[1]],[x+1,y]]
        boundary_line_t = [[xt,yt],[self.robot.ego_veh_goal[0]+50,self.robot.ego_veh_goal[1]],[self.robot.ego_veh_goal[0]+1+50,self.robot.ego_veh_goal[1]],[xt+1,yt]]
        boundaries = [boundary_line, boundary_line_t]
        for boundary in boundaries:
            if self.controller.intersection_check(boundary, self.world.obstacle) or self.controller.intersection_check(boundary, self.world.vehicle1):
                return False
        return True

    def rob_move(self,x,y,theta,xt,yt,theta_t,vel):
        diff_heading = 0
        theta = (theta+360)%360
        theta_t = (theta_t+360)%360
        if theta_t == 0:
            theta_t= 360
        rev_p = 0
        t_p = 0
        o_p = 0
        dx = self.robot.ego_veh_goal[0]-x
        dy = self.robot.ego_veh_goal[1]-y
        distance =  math.sqrt(dx ** 2 + dy ** 2)
        dx2 = (self.robot.ego_veh_goal[0] - self.robot.veh_width) - (x + self.robot.veh_width * math.cos(theta * (math.pi/180)))
        dy2 = (self.robot.ego_veh_goal[1] + self.robot.veh_height) - (y + self.world.veh_height * math.sin(theta * (math.pi/180)))
        distance += math.sqrt(dx2 ** 2 + dy2 ** 2)                
        if self.path_available(x,y,xt,yt) and not(x > self.robot.ego_veh_goal[0]-5 and y > self.robot.ego_veh_goal[1]-5 and x <self.robot.ego_veh_goal[0]+5 and y <self.robot.ego_veh_goal[1]+5):
            diff_heading = abs((360 + (math.atan2(y-self.robot.ego_veh_goal[1],x-self.robot.ego_veh_goal[0]))*(180/math.pi))%360 - theta+180)
        else:
            diff_heading = 180
        if self.controller.intersection_check([[x-15,y],[x+200*math.cos(theta*(math.pi/180))-15,y+200*math.sin(theta*(math.pi/180))],[15+x+200*math.cos(theta*(math.pi/180)),y+200*math.sin(theta*(math.pi/180))],[x+15,y]],self.world.obstacle):
            o_p +=10
        if self.controller.intersection_check([[x-15,y],[x+200*math.cos(theta*(math.pi/180))-15,y+200*math.sin(theta*(math.pi/180))],[15+x+200*math.cos(theta*(math.pi/180)),y+200*math.sin(theta*(math.pi/180))],[x+15,y]],self.world.obstacle):
            o_p +=10
        if vel < 0:
            rev_p = 1
        if abs(theta-theta_t)>15 and not(x > self.robot.ego_veh_goal[0]-5 and y > self.robot.ego_veh_goal[1]-5 and x <self.robot.ego_veh_goal[0]+5 and y <self.robot.ego_veh_goal[1]+5):
            t_p = 5
        hurestic = distance + diff_heading + rev_p + t_p + o_p 
        return hurestic

  
    def AStar_path_planner(self):
        visited_nodes = []
        #Initialise list and dictionaries to keep track of g_score and f_score 
        g_val = {}
        f_val=[]
        start = [self.robot.ego_veh_start[0],self.robot.ego_veh_start[1],self.robot.ego_veh_start[2],self.robot.trailer_start[0],self.robot.trailer_start[1],self.robot.trailer_start[2]]
        print("Starting Node: {f}".format(f=start))
        f = 0
        g = 0
        path = [start]
        #A priority queue is initialized and the starting node is added to it with f and g values initialized to 0 and an empty path.
        prior_queue = PriorityQueue()
        prior_queue.put((f,g,start,path))
      
        while not prior_queue.empty():
          #Inside the loop, the node with the lowest f value is retrieved from the priority queue and its coordinates, g_score, f_score and path are extracted.
          #get the next item from the priority queue 
          f_score, g_score, curr, path = prior_queue.get()
          #check if the current node has already been visited
          if not (self.check_visited([round(curr[0]),round(curr[1]),round(curr[2]),round(curr[3]),round(curr[4]),round(curr[5])],visited_nodes)):
              visited_nodes.append([round(curr[0]),round(curr[1]),round(curr[2]),round(curr[3]),round(curr[4]),round(curr[5])])
              # check if the current node is the goal
              ego_x, ego_y, ego_theta = self.robot.ego_veh_goal
              if round(curr[0]) <= ego_x+5 and round(curr[0]) >= ego_x-5 and round(curr[1]) <= ego_y+5 and round(curr[1]) >= ego_y-5 and curr[2] <= ego_theta+15 and curr[2] >= ego_theta-15:
                  return path
            
              # get the neighbors of the current node
              neighbours = self.get_neighbours(curr[0],curr[1],curr[2],curr[3],curr[4],curr[5])
              for neighbour in neighbours:
                  #calculate the distance cost from the current node to the neighbor
                  #dist_cost = math.sqrt((pow(curr[0]-curr[1],2)+pow(neighbour[0]-neighbour[1],2)))
                  #The g_score and f_score of each neighbor is calculated and added to the priority queue.
                  #calculate the g value for the neighbor
                  g_val = g_score + (0.1*self.dist_cost(curr[0],curr[1],neighbour[0],neighbour[1]))
                  # calculate the f value for the neighbor
                  f_score = g_val +(0.9*self.rob_move(neighbour[0],neighbour[1],neighbour[2],neighbour[3],neighbour[4],neighbour[5],neighbour[6]))
                  # add the neighbor to the priority queue
                  prior_queue.put((f_score,g_val,neighbour, path + [neighbour]))
        return path

if __name__ == "__main__":
    Ack_rob = Robot()
    world = World(Ack_rob)
    Ack_controller = Controller(Ack_rob,world)
    Ack_planner = Planner(Ack_rob,world,Ack_controller) 
    path_values_calculated = Ack_planner.AStar_path_planner()
    print("trajectory",path_values_calculated)
    print("TRAJECTORY FOUND. I WILL PARK THE CAR DONT WORRY !")

 
    for point in path_values_calculated:
        world.environment(point[0],point[1],point[2],point[3],point[4],point[5])
        plt.pause(0.00001)
      

#Plotting the Path taken by the Vehcile 
plt.figure("Position Plot of Truck_Trailer_ackermann_steering")
array = []
array_y = []
for point in path_values_calculated:
    array.append([point[0]])
    array_y.append([point[1]])

plt.plot(array,array_y,'g--')
plt.show()  
