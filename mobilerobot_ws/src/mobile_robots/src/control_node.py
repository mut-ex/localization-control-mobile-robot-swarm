#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import socket
import time
import thread
import threading
import numpy as np
from datetime import datetime
import signal
import sys

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import time
from threading import Thread

TD = 3

''' Constants '''

M_PI = 3.14159265358979323846264338327950288

#TCP port used for communicating with the mobile robots
TCP_PORT = 8150 

#Acceptable minimum error in translation
TRANSLATION_ERROR = 25

#Acceptable minimum error in rotation
ROTATION_ERROR = 12

WAYPOINTS = 2

#The number of mobile robots
ROBOTS_NUM = 8

#Minimum clearance to be ensured when going around obstacles
CLEARANCE = 105

#Generic radius of obstacles
OBSTACLE_RADIUS = 105

#Global container for all MobileRobot objects
global_planner = None

mobile_robots = [None] * ROBOTS_NUM

#Static IP address of mobile robots
mobile_robot_ips = [None] * ROBOTS_NUM
mobile_robot_ips[0] = "192.168.0.101"
mobile_robot_ips[1] = "192.168.0.102"
mobile_robot_ips[2] = "192.168.0.103"
mobile_robot_ips[3] = "192.168.0.104"

mobile_robot_ips[4] = "192.168.0.105"
mobile_robot_ips[5] = "192.168.0.109"
mobile_robot_ips[6] = "192.168.0.107"
mobile_robot_ips[7] = "192.168.0.108"

#Initialize semaphores for each mobile robot (used to synchronize with the perception node)
event = [None] * ROBOTS_NUM
event[0] = threading.Event()
event[1] = threading.Event()
event[2] = threading.Event()
event[3] = threading.Event()
event[4] = threading.Event()
event[5] = threading.Event()
event[6] = threading.Event()
event[7] = threading.Event()

#Look up table to obtain the mobile robot ID associated with each ArUco marker ID
mobile_lookup_table = {}
mobile_lookup_table[0] = 0
mobile_lookup_table[1] = 1
mobile_lookup_table[336] = 2
mobile_lookup_table[340] = 3
mobile_lookup_table[4] = 4
mobile_lookup_table[5] = 5
mobile_lookup_table[325] = 6
mobile_lookup_table[21] = 7

#Lambda function to get the current time in milliseconds
current_milli_time = lambda: int(round(time.time() * 1000))


''' Generic function definitions '''

#Function to ensure that all open TCP connections are closed prior to terminating the program
def clean_exit(i):
    print('Cleaning up...')
    if ROBOTS_NUM > 0:
        for n in xrange(0, i-1):
            mobile_robots[n].stop("exit", [0])
            mobile_robots[n].shutdown()
    sys.exit(0)

#Callback function for SIGTSTP to ensure clean exit
def signal_handler(signal, frame):
    print('Cleaning up...')
    if i > 0:
        for n in xrange(0, i-1):
            mobile_robots[n].stop("exit", [0])
            mobile_robots[n].shutdown()
    sys.exit(0)

def rad2deg(angle):
    return (angle*180)/M_PI;

def deg2rad(angle):
    return (angle*M_PI)/180;

#Callback function to unpack data from the perception node
def callback(data):
    temp_ids = []
    if len(data.data) == 0:
        print "No new localization data!"
        return

    #print "New localization data received!"
    for x in xrange(0, (len(data.data)/5)):
        temp_id = int(data.data[0 + 5*x])
        temp_x = int(data.data[1 + 5*x])
        temp_y = int(data.data[2 + 5*x])
        temp_theta = int(data.data[3 + 5*x])
        temp_ts = int(data.data[4 + 5*x])

        global_planner.add_obstacle(mobile_lookup_table[temp_id], temp_x, temp_y)
        if mobile_lookup_table[temp_id] > ROBOTS_NUM - 1:
            continue

        mobile_robots[mobile_lookup_table[temp_id]].update_current(temp_x, temp_y, temp_theta, temp_ts)
        #print "id = ", temp_id, "x = ", temp_x, "mm y = ", temp_y, "mm Theta(Z) = ", temp_theta
        
        event[mobile_lookup_table[temp_id]].set()
        event[mobile_lookup_table[temp_id]].clear()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.init_node('control_node', anonymous=True)

    rospy.Subscriber("chatter", Float64MultiArray, callback)
    
    # spin() simply keeps python from exiting until this node is stopped

#Function to assign robots to formation points based on proximity
def plan_formation(formation_points = [], robot_positions = []):
    point_distance_to_robot = {}
    formation_plan = {}
    i = 0
    for point in formation_points:
        point_distance_to_robot[i] = {}
        j = 0
        for robot in robot_positions:
            point_distance_to_robot[i][j] = np.floor(euclidean_distance(point[0], point[1], robot[0], robot[1]))
            j += 1
        i += 1

    point_i = 0

    for distance_dict in point_distance_to_robot:
        closest_robot = 0
        min_dist = 999999
        for robot in point_distance_to_robot[distance_dict]:
            print robot, point_distance_to_robot[distance_dict][robot]
            if point_distance_to_robot[distance_dict][robot] < min_dist:
                min_dist = point_distance_to_robot[distance_dict][robot]
                closest_robot = robot
        print "Result =", closest_robot, min_dist
        #formation_plan[closest_robot] = [formation_points[distance_dict], min_dist]
        formation_plan[closest_robot] = formation_points[distance_dict]
        for update_distance_dict in point_distance_to_robot:
            #print point_distance_to_robot[update_distance_dict][closest_robot]
            del point_distance_to_robot[update_distance_dict][closest_robot]

        print "\n"

    print formation_plan
    return formation_plan

#Function for sampling i equidistant points lying on a circle of given radius and center
def sample_circle(center_x, center_y, radius, samples):
    coords = []
    for i in xrange(0, samples):
        a = deg2rad(i*(360/samples))
        coords.append([np.floor(center_x + radius*np.cos(a)),np.floor(center_y + radius*np.sin(a)), 180 - i*(360/samples)])
    return coords

#Function for calculating the gradient of a line
def calculate_m(curr_x, curr_y, des_x, des_y):
    #print "Gradient for:", curr_x, curr_y, des_x, des_y
    return float(des_y - curr_y)/float(des_x - curr_x)

#Function for calculating the y-intercept of a line
def calculate_c(x, y, m):
    return y - m*x

#Function for caclulating the euclidean distance between two points
def euclidean_distance(curr_x, curr_y, des_x, des_y):
    return (((des_x - curr_x)**2) + ((des_y - curr_y)**2))**0.5

#Function for detecting if there is a collision occuring between an obstacle of a certain radius and a line between two points
def detect_collision(curr_x, curr_y, curr_theta, des_x, des_y, obstacle_x, obstacle_y, obstacle_radius):
    if curr_x != des_x and curr_y != des_y:
        AD_m = calculate_m(curr_x, curr_y, des_x, des_y)
        AD_c = calculate_c(curr_x, curr_y, AD_m)

        LineC_m = -1/AD_m
        LineC_c = calculate_c(obstacle_x, obstacle_y, LineC_m)

        X_intersect = (LineC_c - AD_c)/(AD_m - LineC_m)
        Y_intersect = LineC_m*X_intersect + LineC_c
    else:
        if curr_y == des_y:
            X_intersect = obstacle_x
            Y_intersect = curr_y
        elif curr_x == des_x:
            X_intersect = curr_x
            Y_intersect = obstacle_y

    L = euclidean_distance(X_intersect, Y_intersect, obstacle_x, obstacle_y)

    if L > obstacle_radius:
        print "\n[v] No collision detected"
        return [None, None]

    elif L <= obstacle_radius:
        print "\n[!] Collision detected with line!"
        if des_y > curr_y:
            print "Destination above"
            if (curr_x == des_x) and (obstacle_y > curr_y and obstacle_y < des_y):
                print "   Above...going from left."
                detour_x = obstacle_x + CLEARANCE
                detour_y = obstacle_y
                return [np.floor(detour_x), np.floor(detour_y)]

            if (obstacle_x < curr_x and obstacle_x > des_x):
                #print "[!x] Collision with line segment!"
                if obstacle_x >= X_intersect:
                    print "   Better to go from right."
                    detour_x = obstacle_x - (CLEARANCE*np.cos(deg2rad(curr_theta - 90)))
                    detour_y = obstacle_y - (CLEARANCE*np.sin(deg2rad(curr_theta - 90)))
                    return [np.floor(detour_x), np.floor(detour_y)]
                elif obstacle_x < X_intersect:
                    print "   Better to go from left."
                    detour_x = obstacle_x + (CLEARANCE*np.cos(deg2rad(curr_theta - 90)))
                    detour_y = obstacle_y + (CLEARANCE*np.sin(deg2rad(curr_theta - 90)))
                    return [np.floor(detour_x), np.floor(detour_y)]
            if (obstacle_x > curr_x and obstacle_x < des_x):
                if obstacle_x >= X_intersect:
                    print "   Better to go from right."
                    detour_x = obstacle_x - (CLEARANCE*np.cos(deg2rad(90 - curr_theta)))
                    detour_y = obstacle_y + (CLEARANCE*np.sin(deg2rad(90 - curr_theta)))
                    return [np.floor(detour_x), np.floor(detour_y)]
                elif obstacle_x < X_intersect:
                    print "   Better to go from left."
                    detour_x = obstacle_x + (CLEARANCE*np.cos(deg2rad(90 - curr_theta)))
                    detour_y = obstacle_y - (CLEARANCE*np.sin(deg2rad(90 - curr_theta)))
                    return [np.floor(detour_x), np.floor(detour_y)]

        elif des_y < curr_y:
            print "Destination below"
            if (curr_x == des_x) and (obstacle_y < curr_y and obstacle_y > des_y):
                print "   Above...going from right."
                detour_x = obstacle_x + CLEARANCE
                detour_y = obstacle_y
                return [np.floor(detour_x), np.floor(detour_y)]

            if (obstacle_x < curr_x and obstacle_x > des_x):
                #print "[!x] Collision with line segment!"
                if obstacle_x >= X_intersect:
                    print "   tlBetter to go from left."
                    detour_x = obstacle_x - (CLEARANCE*np.cos(deg2rad(abs(curr_theta)-90)))
                    detour_y = obstacle_y + (CLEARANCE*np.sin(deg2rad(abs(curr_theta)-90)))
                    return [np.floor(detour_x), np.floor(detour_y)]
                elif obstacle_x < X_intersect:
                    print "   tlBetter to go from right."
                    detour_x = obstacle_x + (CLEARANCE*np.cos(deg2rad(abs(curr_theta)-90)))
                    detour_y = obstacle_y - (CLEARANCE*np.sin(deg2rad(abs(curr_theta)-90)))
                    return [np.floor(detour_x), np.floor(detour_y)]

            if (obstacle_x > curr_x and obstacle_x < des_x):
                if obstacle_x >= X_intersect:
                    print "   trBetter to go from left."
                    detour_x = obstacle_x + (CLEARANCE*np.cos(deg2rad(90 + abs(curr_theta))))
                    detour_y = obstacle_y - (CLEARANCE*np.sin(deg2rad(90 + abs(curr_theta))))
                    return [np.floor(detour_x), np.floor(detour_y)]
                elif obstacle_x < X_intersect:
                    print "   trBetter to go from right."
                    detour_x = obstacle_x - (CLEARANCE*np.cos(deg2rad(90 + abs(curr_theta))))
                    detour_y = obstacle_y + (CLEARANCE*np.sin(deg2rad(90 + abs(curr_theta))))
                    return [np.floor(detour_x), np.floor(detour_y)]

        elif des_y == curr_y:
            if (obstacle_x < curr_x and obstacle_x > des_x) or (obstacle_x > curr_x and obstacle_x < des_x):
                print "   Right or left..."
                detour_x = obstacle_x 
                detour_y = obstacle_y + CLEARANCE
                return [np.floor(detour_x), np.floor(detour_y)]

        if (euclidean_distance(obstacle_x, obstacle_y, des_x, des_y) <= OBSTACLE_RADIUS) and (euclidean_distance(curr_x, curr_y, des_x, des_y) <= OBSTACLE_RADIUS):
            return [0, 0]
        else:
            return [None, None]

''' Class for 2D Bezier curve based path planning '''
class BezierCurve(object):
    def __init__(self, x_points = [], y_points = []):
            self.B_x = x_points
            self.B_y = y_points
    
            self.P_x = []
            self.P_y = []
    
            self.n = len(x_points)
    
            self.t_array = np.arange(0, 1, 0.01)
    
    #Function for generating a Bezier curve using the associated control points
    def generate_curve(self):
        self.P_x = []
        self.P_y = []

        curr_J = 0
        P_sum_x = 0
        P_sum_y = 0

        for t in self.t_array:
            for i in xrange(0, self.n):
                curr_J = math.factorial(self.n-1) / (math.factorial(i)*math.factorial(self.n-1 - i))
                curr_J = curr_J*(t**i)
                curr_J = curr_J*(1-t)**(self.n-1 - i)

                P_sum_x += self.B_x[i]*curr_J
                P_sum_y += self.B_y[i]*curr_J

            self.P_x.append(P_sum_x)
            self.P_y.append(P_sum_y)

            P_sum_x = 0
            P_sum_y = 0

    #Function for printing all the points in the generated Bezier curver
    def print_curve(self):
        print "X(t) = ", self.P_x
        print "Y(t) = ", self.P_y

    #Function for updating the control points
    def update_points(self, x = [], y = []):
        self.B_x = x
        self.B_y = y

        self.n = len(x)

    #Function for generating waypoints by sampling the generated Bezier curve
    def generate_waypoints(self):
        w_x = []
        w_y = []
        s = 2
        #print "Total points: ", len(self.P_x)
        while (s <= len(self.P_x)):
            w_x.append(self.P_x[s])
            w_y.append(self.P_y[s])
            s += 20
        w_x.append(self.P_x[len(self.P_x)-1])
        w_y.append(self.P_y[len(self.P_y)-1])

        return w_x, w_y

    #Function for displaying the Bezier curve in real-time (kind of buggy)
    def display_curve(self):
        mpl.rcParams['toolbar'] = 'None'

        plt.ion()

        img = plt.imread("backdrop.jpg")
        fig = plt.figure(frameon=False, figsize=(16, 9))
        ax = fig.add_axes([0, 0, 1, 1])
        ax.axis('off')

        self.generate_curve()

        w_x = []
        w_y = []
        s = 2
        while (s <= len(self.P_x)):
            w_x.append(self.P_x[s])
            w_y.append(self.P_y[s])
            s += 10


        scatter1, = ax.plot(w_x, w_y, "o", zorder = 3, color = 'b', markersize = 10)
        curve1, = ax.plot(self.P_x, self.P_y, zorder = 2, linewidth=3, color = 'm')
        line1, = ax.plot(self.B_x, self.B_y, zorder = 1, linewidth=2, color = 'c')

        box1 = ax.add_patch(patches.Rectangle((self.B_x[0], self.B_y[0]), 30, 30, color = 'y', angle = 0))

        implot = ax.imshow(img, zorder = 0, extent=[640, -640, -360, 360])
        
        while True:
            #print "Tick"
            self.generate_curve()

            curve1.set_xdata(self.P_x)
            curve1.set_ydata(self.P_y)

            line1.set_xdata(self.B_x)
            line1.set_ydata(self.B_y)

            box1.set_x(self.B_x[0] - 50/2)
            box1.set_y(self.B_y[0] - 50/2)

            w_x = []
            w_y = []
            s = 2
            #print "Total points: ", len(self.P_x)
            while (s <= len(self.P_x)):
                w_x.append(self.P_x[s])
                w_y.append(self.P_y[s])
                s += 20
            w_x.append(self.P_x[len(self.P_x)-1])
            w_y.append(self.P_y[len(self.P_y)-1])

            scatter1.set_xdata(w_x)
            scatter1.set_ydata(w_y)

            #box1.set_theta(45)

            fig.canvas.draw()
            time.sleep(0.125)


''' Class to track all obstacles, has potential for added functionality '''
class GlobalPlanner(object):
    def __init__(self):
        self.obstacles = {}

    def add_obstacle(self, obstacle_id, x_pos, y_pos):
        self.obstacles[obstacle_id] = [x_pos, y_pos]
        #print self.obstacles

    def get_obstacles(self):
        return self.obstacles


''' Class for the mobile robots'''
class MobileRobot(object):
    def __init__(self, my_ip, robot_id, robot_planner, debug_flg):
        self.ip_address = my_ip
        if int(debug_flg) == 0:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                self.s.connect((self.ip_address, TCP_PORT))
                self.s.sendall('t1')
                print "Connection established with mobile robot @", self.ip_address
            except:
                raise Exception

        self.my_id = robot_id
        self.my_planner = robot_planner

        self.obstacles = {}
            
        self.curr_x = None
        self.curr_y = None
        self.curr_theta = None
        self.curr_ts = None

        self.des_x = None
        self.des_y = None
        self.des_theta = None

        self.debug_log = []
        self.plot_log = []

        self.MyBezier = BezierCurve([],[])

        self.flag = 0
        self.on_detour = []

    #Function for initializing Bezier curve object for this mobile robot
    def init_bezier():
        BezierCurve([self.curr_x, self.curr_x, self.des_x,self.des_x], self.curr_y, self.curr_y, self.des_y, self.des_y)
        

    def update_obstacles(self):
        temp_obstacles = self.my_planner.get_obstacles()

        for obstacles in temp_obstacles:
            #print obstacles
            self.obstacles[obstacles] = temp_obstacles[obstacles]

        try:
            del self.obstacles[self.my_id]
        except KeyError:
            pass

        # if cmp(temp_obstacles, self.obstacles) == 0:
        #     print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11Obstacles haven't moved"
        #     print self.obstacles
        #     return 0
        # else:
        #     self.obstacles = temp_obstacles
        #     print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11Obstacles moved"
        #     print self.obstacles
        #     return 1

    def log_it(self, function_name, commandname, target = []):
        log_str = datetime.utcnow().strftime("%H:%M:%S.%f")[:-3] + ", "
        log_str += function_name + ", "
        if len(target) > 1:
            log_str += "target_x = " + str(target[0]) + ", target_y = " + str(target[1]) + ", "
        else:
            log_str += "target_angle = " + str(target[0]) + ", "
        log_str += commandname + ", "
        log_str += "x = " + str(self.curr_x) + ", "
        log_str += "y = " + str(self.curr_y) + ", "
        log_str += "theta = " + str(self.curr_theta)
        self.debug_log.append(log_str)


        plot_log_str = str(current_milli_time()) + "," + str(self.curr_x) + "," + str(self.curr_y) + "," + str(self.curr_theta)
        self.plot_log.append(plot_log_str)

    def log_flush(self):
        my_time = time.strftime("%H-%M-%S", time.gmtime())
        logfile = open("mobile_robot_log_" + my_time +".txt", 'w')
        for item in self.debug_log:
            logfile.write("%s\n" % item)
        logfile.close()

        logfile = open("mobile_robot_plot_log_" + my_time +".txt", 'w')

        for item in self.plot_log:
            logfile.write("%s\n" % item)
        logfile.close()

        del self.debug_log[:]
        del self.plot_log[:]

    def shutdown(self):
        self.s.shutdown(socket.SHUT_RDWR)
        self.s.close()

    def update_current(self, x, y, theta, ts):
        self.curr_x = x
        self.curr_y = y
        self.curr_theta = theta
        self.curr_ts = ts

    def get_current(self):
        return [self.curr_x, self.curr_y, self.curr_theta]

    def update_destination(self, des_position = []):
        self.des_x = des_position[0]
        self.des_y = des_position[1]
        self.des_theta = des_position[2]

    def print_current(self):
        print "\nCurrent[",self.my_id,"] =", self.curr_x, self.curr_y, self.curr_theta, "TS_then:", self.curr_ts

    def print_destination(self):
        print "\nDestination[",self.my_id,"] =", self.des_x, self.des_y, self.des_theta

    def move_forward(self, function_name, target = []):
        self.s.sendall("1121")
        self.log_it(function_name, "move_forward", target)

    def move_back(self, function_name, target = []):
        self.s.sendall("1222")
        self.log_it(function_name, "move_back", target)

    def turn_left(self, function_name, target = []):
        self.s.sendall("1221")
        self.log_it(function_name, "turn_left", target)

    def turn_right(self, function_name, target = []):
        self.s.sendall("1122")
        self.log_it(function_name, "turn_right", target)

    def pivot_turn_left(self, function_name, target = []):
        self.s.sendall("1021")
        self.log_it(function_name, "turn_left", target)

    def pivot_turn_right(self, function_name, target = []):
        self.s.sendall("1120")
        self.log_it(function_name, "turn_right", target)

    def stop(self, function_name, target = []):
        self.s.sendall("1020")
        self.log_it(function_name, "stop", target)

    def log_position(self):
        f = open("x_coordinates.txt", "a")
        f.write(str(self.curr_x) + "\n")
        f.close()

        f = open("y_coordinates.txt", "a")
        f.write(str(self.curr_y) + "\n")
        f.close()

    def euclidean_distance(self, target_x, target_y):
        return (((target_x - self.curr_x)**2) + ((target_y - self.curr_y)**2))**0.5

    def move_to_xy(self, target_x, target_y):
        #path_m = (target_y - self.curr_y) / (target_x - self.curr_x)
        ##path_c = target_y - path_m*(target_x)
        while True:
            event[self.my_id].wait()
            if self.flag == 0:
                self.update_obstacles()
                detour_coordinates = []
                for obstacle in self.obstacles:
                    if len(self.on_detour) == 0:
                        detour_temp = detect_collision(self.curr_x, self.curr_y, self.curr_theta, self.des_x, self.des_y, self.obstacles[obstacle][0], self.obstacles[obstacle][1], OBSTACLE_RADIUS)
                    else:
                        detour_temp = detect_collision(self.curr_x, self.curr_y, self.curr_theta, target_x, target_y, self.obstacles[obstacle][0], self.obstacles[obstacle][1], OBSTACLE_RADIUS)
        
                    if detour_temp == [0, 0]:
                        print "[",self.my_id,"] Obstacle near destination position"
                        self.stop("move_to_xy", [target_x, target_y])
                        while 1:
                            z = 0
                            event[self.my_id].wait()
                            self.update_obstacles()
                            for obstacle in self.obstacles:
                                if len(self.on_detour) == 0:
                                    detour_temp = detect_collision(self.curr_x, self.curr_y, self.curr_theta, self.des_x, self.des_y, self.obstacles[obstacle][0], self.obstacles[obstacle][1], OBSTACLE_RADIUS)
                                else:
                                    detour_temp = detect_collision(self.curr_x, self.curr_y, self.curr_theta, target_x, target_y, self.obstacles[obstacle][0], self.obstacles[obstacle][1], OBSTACLE_RADIUS)
                    
                                if detour_temp == [0, 0]:
                                    print "[",self.my_id,"] Obstacle still near destination position"
                                    z = 1
                            
                            if z == 0:
                                print "[",self.my_id,"] Obstacle moved"
                                break
                        event[self.my_id].wait()
                        break

                    if detour_temp != [None, None]:
                        self.on_detour.append(1)
                        print "Detour depth =", len(self.on_detour)
                        self.stop("move_to_xy", [target_x, target_y])
                        detour_coordinates.append(detour_temp)

                        print "List of detours:", detour_coordinates

                        detour_distance = []
                        for detour_coordinate in detour_coordinates:
                            detour_distance.append(self.euclidean_distance(detour_coordinate[0], detour_coordinate[1]))

                        print "Distance to detours:", detour_distance

                        closest_detour = sorted(range(len(detour_distance)), key=lambda k:detour_distance[k])
                        
                        print "Closet detour at", closest_detour[0], detour_coordinates[closest_detour[0]][0], detour_coordinates[closest_detour[0]][1]

                        if euclidean_distance(detour_coordinates[closest_detour[0]][0], detour_coordinates[closest_detour[0]][1], target_x, target_y) <= 5:
                            self. flag = 1;
                            continue
                        print "Rotating towards detour co-ordinates"
                        self.rotate_to_xy(detour_coordinates[closest_detour[0]][0], detour_coordinates[closest_detour[0]][1])
                        print "Moving towards detour co-ordinates"
                        self.move_to_xy(detour_coordinates[closest_detour[0]][0], detour_coordinates[closest_detour[0]][1])
                        print "Arrived at detour co-ordinates"
                        self.on_detour.pop()
                        self.flag == 0
                        self.print_destination()
                        event[self.my_id].wait()
                    else:
                        print "My path is clear!"
                        self.print_destination()
            print "im here now"
            target_angle = self.get_target_angle(target_x, target_y)

            if abs(target_angle-self.curr_theta) > ROTATION_ERROR:
                #self.stop("move_to_xy", [target_x, target_y])
                self.special_rotate_to_xy(target_x, target_y)

            event[self.my_id].wait()

            #self.print_current()
            #print "> Target co-ordinates = ",target_x, target_y
            #print "Distance from target = ", self.euclidean_distance(target_x, target_y)

            if self.euclidean_distance(target_x, target_y) <= TRANSLATION_ERROR:
                self.stop("move_to_xy", [target_x, target_y])
                break
            else:
                self.move_forward("move_to_xy", [target_x, target_y])

    def get_departure_controlpoint(self, del_x):
        if self.curr_theta > 0 and self.curr_theta < 90:
            d_cp_x = self.curr_x + del_x
            d_cp_y = np.floor(d_cp_x*np.tan(deg2rad(self.curr_theta)) - self.curr_y)

        elif self.curr_theta > 90 and self.curr_theta < 180:
            d_cp_x = self.curr_x - del_x
            d_cp_y = np.floor(d_cp_x*np.tan(deg2rad(180-self.curr_theta)) - self.curr_y)

        elif self.curr_theta < 0 and self.curr_theta > -90:
            d_cp_x = self.curr_x + del_x
            d_cp_y = np.floor(self.curr_y - d_cp_x*np.tan(deg2rad(abs(self.curr_theta))))

        elif self.curr_theta < -90 and self.curr_theta > -180:
            d_cp_x = self.curr_x - del_x
            d_cp_y = np.floor(self.curr_y - d_cp_x*np.tan(deg2rad(180-abs(self.curr_theta))))

        elif self.curr_theta == 0:
            d_cp_x = self.curr_x + del_x
            d_cp_y = self.curr_y

        elif self.curr_theta == 180 or self.curr_theta == -180:
            d_cp_x = self.curr_x - del_x
            d_cp_y = self.curr_y

        elif self.curr_theta == 90:
            d_cp_x = self.curr_x
            d_cp_y = self.curr_y + del_x

        elif self.curr_theta == -90:
            d_cp_x = self.curr_x
            d_cp_y = self.curr_y - del_x

        else:
            print "Whaaaaaaat???", self.curr_x, self.curr_y

        return (d_cp_x, d_cp_y)

    def get_target_angle(self, target_x, target_y):
        if target_x == self.curr_x:
            if target_y > self.curr_y:
                target_angle = 90

            elif target_y < self.curr_y:
                target_angle = -90


        elif target_y == self.curr_y:
            if target_x > self.curr_x:
                target_angle = 0

            elif target_x < self.curr_x:
                target_angle = 180


        elif self.curr_y > target_y:
            if self.curr_x < target_x:
                target_angle = -np.arctan(deg2rad(abs(target_y - self.curr_y)) / deg2rad(abs(target_x - self.curr_x)))
                target_angle = np.floor(rad2deg(target_angle))

            else:
                target_angle = np.arctan(deg2rad(abs(target_y - self.curr_y)) / deg2rad(abs(target_x - self.curr_x)))
                target_angle = -np.floor(180 - rad2deg(target_angle))


        elif self.curr_y < target_y:
            if self.curr_x < target_x:
                target_angle = np.arctan(deg2rad(abs(target_y - self.curr_y)) / deg2rad(abs(target_x - self.curr_x)))
                target_angle = np.floor(rad2deg(target_angle))

            else:
                target_angle = np.arctan(deg2rad(abs(target_y - self.curr_y)) / deg2rad(abs(target_x - self.curr_x)))
                target_angle = np.floor(180 - rad2deg(target_angle))


        return target_angle

    def special_rotate_to_xy(self, target_x, target_y):
        target_angle = self.get_target_angle(target_x, target_y)

        rot_error = 0
        kp = 0.010417
        while True:
            event[self.my_id].wait()

            rot_error = target_angle - self.curr_theta

            if rot_error <= -180:
                print "ag There is a shorter way! Prev:", rot_error
                rot_error = 360 + rot_error
                print "There is a shorter way! New:", rot_error
            if rot_error >= 180:
                print "ag There is a shorter way! Prev:", rot_error
                rot_error = rot_error - 360
                print "There is a shorter way! New:", rot_error

            self.print_current()
            print "> Target angle = ", target_angle
            print "Error = ", rot_error

            # self.turn_right("rotate_to_xy", [target_angle])
            # time.sleep(0.125)
            # self.stop("rotate_to_xy", [target_angle])
            if abs(rot_error) <= ROTATION_ERROR:
                print "Acceptable"
                self.stop("rotate_to_xy", [target_angle])
                #time.sleep(0.420)
                break

            elif rot_error < 0:
                print "Lesser"
                self.turn_left("rotate_to_xy", [target_angle])
                print "Sleep time = ", abs(rot_error)*kp
                if abs(rot_error)*kp < 0.115:
                    pass
                time.sleep(abs(rot_error)*kp)
                # tmp = int(np.floor(abs(rot_error)/12))
                # print "Chunks = ",tmp
                # for z in xrange(tmp):
                #     time.sleep(0.115)

                self.stop("rotate_to_xy", [target_angle])

            elif rot_error > 0:
                print "Greater"
                self.turn_right("rotate_to_xy", [target_angle])
                print "Sleep time = ", abs(rot_error)*kp
                if abs(rot_error)*kp < 0.115:
                    pass
                time.sleep(abs(rot_error)*kp)
                # tmp = int(np.floor(abs(rot_error)/12))
                # print "Chunks = ",tmp
                # for z in xrange(tmp):
                #     time.sleep(0.115)

                self.stop("rotate_to_xy", [target_angle])
            
            time.sleep(0.420/TD)

    def rotate_to_xy(self, target_x, target_y):
        event[self.my_id].wait()

        target_angle = self.get_target_angle(target_x, target_y)

        rot_error = 0
        kp = 0.010417
        iterations = 0
        #print "Told to rotate to", target_angle
        while True:
            #print "\n\nWaiting...",iterations
            iterations += 1

            event[self.my_id].wait()

            #print "New position data received!"

            rot_error = target_angle - self.curr_theta

            if rot_error <= -180:
                print "ag There is a shorter way! Prev:", rot_error
                rot_error = 360 + rot_error
                print "There is a shorter way! New:", rot_error
            if rot_error >= 180:
                print "ag There is a shorter way! Prev:", rot_error
                rot_error = rot_error - 360
                print "There is a shorter way! New:", rot_error

            self.print_current()
            print "> Target angle = ", target_angle
            print "Error = ", rot_error

            # self.turn_right("rotate_to_xy", [target_angle])
            # time.sleep(0.125)
            # self.stop("rotate_to_xy", [target_angle])
            if abs(rot_error) <= ROTATION_ERROR:
                print "Acceptable"
                #self.stop("rotate_to_xy", [target_angle])
                break

            elif rot_error < 0:
                print "Lesser"
                self.turn_left("rotate_to_xy", [target_angle])
                # print "Sleep time = ", abs(rot_error)*kp
                # time.sleep(abs(rot_error)*kp)
                tmp = int(np.floor(abs(rot_error)/12))
                print "Chunks = ",tmp
                for z in xrange(tmp):
                    time.sleep(0.115)

                self.stop("rotate_to_xy", [target_angle])

            elif rot_error > 0:
                print "Greater"
                self.turn_right("rotate_to_xy", [target_angle])
                # print "Sleep time = ", abs(rot_error)*kp
                # time.sleep(abs(rot_error)*kp)
                tmp = int(np.floor(abs(rot_error)/12))
                print "Chunks = ",tmp
                for z in xrange(tmp):
                    time.sleep(0.115)

                self.stop("rotate_to_xy", [target_angle])
            
            time.sleep(0.420/TD)

    def rotate_to_angle(self, target_angle):
        rot_error = 0
        kp = 0.010417
        iterations = 0
        #print "Told to rotate to", target_angle
        while True:
            #print "\n\nWaiting...",iterations
            iterations += 1

            event[self.my_id].wait()

            #print "New position data received!"

            rot_error = target_angle - self.curr_theta

            if rot_error <= -180:
                print "ag There is a shorter way! Prev:", rot_error
                rot_error = 360 + rot_error
                print "There is a shorter way! New:", rot_error
            if rot_error >= 180:
                print "ag There is a shorter way! Prev:", rot_error
                rot_error = rot_error - 360
                print "There is a shorter way! New:", rot_error

            self.print_current()
            print "> Target angle = ", target_angle
            print "Error = ", rot_error

            # self.turn_right("rotate_to_xy", [target_angle])
            # time.sleep(0.125)
            # self.stop("rotate_to_xy", [target_angle])
            if abs(rot_error) <= ROTATION_ERROR:
                print "Acceptable"
                self.stop("rotate_to_xy", [target_angle])
                break

            elif rot_error < 0:
                print "Lesser"
                self.turn_left("rotate_to_xy", [target_angle])
                print "Sleep time = ", abs(rot_error)*kp
                time.sleep(abs(rot_error)*kp)
                # tmp = int(np.floor(abs(rot_error)/12))
                # print "Chunks = ",tmp
                # for z in xrange(tmp):
                #     time.sleep(0.115)

                self.stop("rotate_to_xy", [target_angle])

            elif rot_error > 0:
                print "Greater"
                self.turn_right("rotate_to_xy", [target_angle])
                print "Sleep time = ", abs(rot_error)*kp
                time.sleep(abs(rot_error)*kp)
                # tmp = int(np.floor(abs(rot_error)/12))
                # print "Chunks = ",tmp
                # for z in xrange(tmp):
                #     time.sleep(0.115)

                self.stop("rotate_to_xy", [target_angle])
            
            time.sleep(0.420/TD)

    def move_to_waypoint(self, wp_x, wp_y):
        print ">>>>> Moving towards next waypoint"
        self.move_to_xy(wp_x, wp_y)

    def path_planning(self):
        event[self.my_id].wait()

        # while 1:
        #     print "------"
        #     event[self.my_id].wait()
        #     self.update_obstacles()
        #     for obstacle in self.obstacles:
        #         detour_temp = detect_collision(self.curr_x, self.curr_y, self.curr_theta, self.des_x, self.des_y, self.obstacles[obstacle][0], self.obstacles[obstacle][1], OBSTACLE_RADIUS)
        #         if detour_temp == [0, 0]:
        #             print "I can't get to it there is"

        #         if detour_temp != [None, None]:
        #             print "Yup obstacle: ", obstacle, detour_temp
        #             #detour_coordinates.append(detour_temp)
        #         else:
        #             print "Not obstacle: ", obstacle
        '''
        path_m = (self.des_y - self.curr_y) / (self.des_x - self.curr_x)
        path_c = self.des_y - path_m*(self.des_x)
        stepsize = (self.des_x - self.curr_x) / WAYPOINTS       
        starting_x = self.curr_x

        for i in xrange(1, WAYPOINTS+1):
            print " !!!!!!!!!! Waypoint Number = ", i

            wp_x = starting_x + stepsize*i
            wp_y = path_m*wp_x + path_c
            print " Next x = ",wp_x," Next y = ",wp_y
            self.move_to_waypoint(wp_x, wp_y)
        print "Arrived at final position! Reorienting..."
        '''
        #self.update_obstacles()
        #Perform 
        #self.rotate_to_angle(90)

        #Bezier curve implementation
        # departure_controlpoints = self.get_departure_controlpoint(150)
        # self.MyBezier.update_points([self.curr_x, departure_controlpoints[0], self.des_x, self.des_x], [self.curr_y, departure_controlpoints[1], self.des_y, self.des_y])
        # self.MyBezier.generate_curve()
        # #self.MyBezier.display_curve()
        # thread.start_new_thread(self.MyBezier.display_curve, ())
        # w_x, w_y = self.MyBezier.generate_waypoints()
        # #self.rotate_to_xy(self.des_x, self.des_y)
        # for wx, wy in zip(w_x, w_y):
        #     departure_controlpoints = self.get_departure_controlpoint(150)
        #     self.MyBezier.update_points([self.curr_x, departure_controlpoints[0], self.des_x, self.des_x], [self.curr_y, departure_controlpoints[1], self.des_y, self.des_y])
        
        #     self.rotate_to_xy(wx, wy)
        #     self.move_to_xy(wx, wy)
        # self.rotate_to_angle(self.des_theta)

        # while 1:
        #     event[self.my_id].wait()
        #     self.update_obstacles()
        #     for obstacle in self.obstacles:
        #         detour_coordinates = detect_collision(self.curr_x, self.curr_y, self.curr_theta, self.des_x, self.des_y, self.obstacles[obstacle][0], self.obstacles[obstacle][1], 180)
        #         if detour_coordinates != [None, None]:
        #             print "\n>>>> Checking collision with:", obstacle
        #             print "obstacle at x=", self.obstacles[obstacle][0]," y=", self.obstacles[obstacle][1]
        #             print "detour x=", detour_coordinates[0], "y=", detour_coordinates[1]
        #             self.rotate_to_xy(detour_coordinates[0], detour_coordinates[1])
        #             self.move_to_xy(detour_coordinates[0], detour_coordinates[1])
        #             print "All done!"
        #             print "LOL", 1/0



        # for point in points:
        # self.update_destination(point)
        self.rotate_to_xy(self.des_x, self.des_y)
        self.move_to_xy(self.des_x, self.des_y)
        self.rotate_to_angle(self.des_theta)


        print "All done!"
        self.print_current()
        self.print_destination()
        #self.log_flush()
    
    def my_test(self):
        self.print_current()

        self.move_forward("my_test", [target_x, target_y])

        for i in xrange(10):
            event[self.my_id].wait()
            self.print_current()
            

        event[self.my_id].wait()   
        self.print_current()
        self.stop("my_test", [target_angle])

if __name__ == '__main__':
    signal.signal(signal.SIGTSTP, signal_handler)

    global_planner = GlobalPlanner()

    print ">> Initializing..."
    for i in xrange(0, ROBOTS_NUM):
        print "Connecting to mobile robot number", i
        try:
            mobile_robots[i] = MobileRobot(mobile_robot_ips[i], i, global_planner, 0)
        except:
            print "Unable to connect to robot number", i
            clean_exit(i)
    
    print ">> Initialization complete..."

    print ">> Starting listener"
    listener()

    test_destinations = [(-114, 196, 178), (16, -54, -28), (83, 354, 63), (320, 179, -117), (-87, 4, 90)]

    test_grid = [(-100, 300, 90), (130, 300, 90), (-100, -15, 90), (130, -15, 90)]

    circle_points = sample_circle(50, 60, 250, ROBOTS_NUM)
    print circle_points
    time.sleep(10)
    robot_positions = []
    for i in xrange (0, ROBOTS_NUM):
        temp_pos = mobile_robots[i].get_current()
        if temp_pos == [None, None, None]:
            print 1/0
        
        robot_positions.append(temp_pos)

    formation_plan = plan_formation(circle_points, robot_positions)

    for robot in formation_plan:
        print "Mobile robot",robot, "will go to", formation_plan[robot]
        mobile_robots[robot].update_destination(formation_plan[robot])

    for i in xrange (0, ROBOTS_NUM):
        thread.start_new_thread(mobile_robots[i].path_planning, ())

    #mobile_robots[0].update_destination(test_destinations[3])
    #mobile_robots[0].update_destination((-545, 370, 45))
    # mobile_robots[0].update_destination(circle_points[0])
    # mobile_robots[1].update_destination(circle_points[1])
    # mobile_robots[2].update_destination(circle_points[2])
    # mobile_robots[3].update_destination(circle_points[3])

    # mobile_robots[0].update_destination(test_grid[0])
    # mobile_robots[1].update_destination(test_grid[1])
    # mobile_robots[2].update_destination(test_grid[2])
    # mobile_robots[3].update_destination(test_grid[3])

    # mobile_robots[0].update_destination(test_destinations[0])
    # mobile_robots[1].update_destination(test_destinations[1])
    # mobile_robots[2].update_destination(test_destinations[2])
    # mobile_robots[3].update_destination(test_destinations[3])
    #time.sleep(10)
    # thread.start_new_thread(mobile_robots[0].path_planning, ())
    # thread.start_new_thread(mobile_robots[1].path_planning, ())
    # thread.start_new_thread(mobile_robots[2].path_planning, ())
    # thread.start_new_thread(mobile_robots[3].path_planning, ())

    #mobile_robots[1].my_test()
    while 1:
        rospy.spin()