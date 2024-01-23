#Datein importieren fürs senden und empfangen von Ros Datein.................................................
import math
import rclpy                                                        #Ros schnittstelle für Python
from rclpy.node import Node
from std_msgs.msg import String                                                
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from rclpy.qos import qos_profile_sensor_data

import numpy as np
from enum import IntEnum
#.............................................................................................................


#Globale Variablen............................................................................................
ROBOT_DIRECTION_FRONT_INDEX = 0
ROBOT_DIRECTION_RIGHT_FRONT_INDEX = 300
ROBOT_DIRECTION_RIGHT_INDEX = 270
ROBOT_DIRECTION_RIGHT_REAR_INDEX = 240
ROBOT_DIRECTION_REAR_INDEX = 180
ROBOT_DIRECTION_LEFT_REAR_INDEX = 120
ROBOT_DIRECTION_LEFT_INDEX = 90
ROBOT_DIRECTION_LEFT_FRONT_INDEX = 60
#.............................................................................................................


#Zustände definieren..........................................................................................
class WallFollowerStates(IntEnum):
    WF_STATE_INVALID = -1,
    WF_STATE_DETECTWALL = 0,
    WF_STATE_DRIVE2WALL = 1,
    WF_STATE_ROTATE2WALL = 2,
    WF_STATE_FOLLOWWALL = 3
#.............................................................................................................


#Node.........................................................................................................
class WallFollower(Node):

    #+++++ Konstruktor +++++
    def __init__ (self):                                            
        super().__init__('Wallfollower')
        self.scan_subscriber = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        #+++++ Variablen +++++
        self.left_dist = 999999.9
        self.leftfront_dist = 999999.9
        self.front_dist = 999999.9
        self.rightfront_dist = 999999.9
        self.right_dist = 999999.9
        self.rear_dist = 999999.9
        self.distances = []

        self.wallfollower_state = WallFollowerStates.WF_STATE_INVALID

        self.forward_speed_wf_slow = 0.05 #m/s
        self.forward_speed_wf_fast = 0.1  #m/s

        self.turning_speed_wf_slow = 0.1  #rad/s
        self.turning_speed_wf_fast = 1.0  #rad/s

        self.dist_thresh_wf = 0.3         #m
        self.dist_hysteresis_wf = 0.02    #m
        self.dist_laser_offset = 0.03     #m

        self.valid_lidar_data = False

        self.timer = self.create_timer(0.2, self.timer_callback)

    #Timer Callback 
    def timer_callback(self):
        if self.valid_lidar_data:        #warten bis gültige Lidar Daten vorhanden sind
            self.follow_wall()


    #Regler, State-Maschine
    def follow_wall(self):

        #neue Nachricht anlegen
        msg = Twist()
        #Geschwindigkeiten null setzen
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0


        #State Invalid
        if self.wallfollower_state == WallFollowerStates.WF_STATE_INVALID:
            print("WF_STATE_DETECTWALL")
            self.wallfollower_state = WallFollowerStates.WF_STATE_DETECTWALL


        #State Detectwall
        elif self.wallfollower_state == WallFollowerStates.WF_STATE_DETECTWALL:
            print("\nDetect Wall\n")
            dist_min = min(self.distances)
            if self.front_dist > dist_min:
                #wenn er knapp bei der wand ist dreh langsam
                if abs(self.front_dist -dist_min < 0.2):
                    print("Roboter dreht\n")
                    msg.angular.z = self.turning_speed_wf_slow
                #sonst schnell
                else:
                    print("Roboter dreht\n")
                    msg.angular.z = self.turning_speed_wf_fast
            else:
                print("WF_STATE_DRIVE2Wall")
                self.wallfollower_state = WallFollowerStates.WF_STATE_DRIVE2WALL


        #State Drive2Wall
        elif self.wallfollower_state == WallFollowerStates.WF_STATE_DRIVE2WALL:
            print("\nDrive to Wall\n")
            fd_thresh = self.dist_thresh_wf + self.dist_laser_offset
            forward_speed_wf = self.calc_linear_speed()
            if self.front_dist > (fd_thresh + self.dist_hysteresis_wf):
                msg.linear.x = forward_speed_wf
            elif self.front_dist < (fd_thresh - self.dist_hysteresis_wf):
                msg.linear.x = -forward_speed_wf
            else:
                turn_direction = self.align_front()
                msg.angular.z = self.turning_speed_wf_slow * turn_direction
                if turn_direction == 0:
                    print("WF_STATE_ROTATE2WALL")
                    #safe the current distances as input for the state ROTATE2WALL
                    self.wallfollower_state_input_dist = self.distances
                    self.wallfollower_state = WallFollowerStates.WF_STATE_ROTATE2WALL


        #State Rotate2Wall
        elif self.wallfollower_state == WallFollowerStates.WF_STATE_ROTATE2WALL:
            print("\nRotate to Wall\n")
            sr = self.wallfollower_state_input_dist[ROBOT_DIRECTION_RIGHT_INDEX]
            if((sr != np.inf) and (abs(self.front_dist - self.dist_laser_offset - sr) > 0.05)) or ((self.front_dist !=  np.inf) and (sr ==np.inf)):
                msg.angular.z = -self.turning_speed_wf_fast
            else:
                turn_direction = self.align_left()
                msg.angular.z = self.turning_speed_wf_slow * turn_direction
                if turn_direction == 0:
                    print("WF_STATE_FOLLOWWALL")
                    self.wallfollower_state = WallFollowerStates.WF_STATE_FOLLOWWALL

        #State FollowWall
        elif self.wallfollower_state == WallFollowerStates.WF_STATE_FOLLOWWALL:
            print("\nFollow Wall\n")



        #Geschwindigkeiten rausschicken
        print(msg)
        self.cmd_vel_publisher.publish(msg)


    #Methode um Geschwindigkeit (langsam oder schnell) zu berechnen
    def calc_linear_speed(self):
        fd_thresh = self.dist_thresh_wf + self.dist_laser_offset
        if self.front_dist > ( 1.2 * fd_thresh):
            forward_speed_wf = self.forward_speed_wf_fast
        else:
            forward_speed_wf = self.forward_speed_wf_slow

        return forward_speed_wf
    

    #Methode um gerade zu Wand ausrichten
    def align_front(self):
        fl = self.distances[ROBOT_DIRECTION_LEFT_FRONT_INDEX]
        fr = self.distances[ROBOT_DIRECTION_RIGHT_FRONT_INDEX]
        if (fl - fr) > self.dist_hysteresis_wf:
            return 1 #turning left
        elif (fr - fl) > self.dist_hysteresis_wf:
            return -1 #turning right
        else:
            return 0 #aligned
        
    #Methode um paralell zu Wand ausrichten
    def align_left(self):
        fl = self.distances[ROBOT_DIRECTION_LEFT_FRONT_INDEX]
        rl = self.distances[ROBOT_DIRECTION_LEFT_REAR_INDEX]
        if (fl - rl) > self.dist_hysteresis_wf:
            return 1 #turning left
        elif (rl - fl) > self.dist_hysteresis_wf:
            return -1 #turning right
        else:
            return 0 #aligned


    #Scan Callback, Lidar Daten holen
    def scan_callback(self, msg):

        self.valid_lidar_data = True
        self.left_dist = msg.ranges[ROBOT_DIRECTION_LEFT_INDEX]
        self.leftfront_dist = msg.ranges[ROBOT_DIRECTION_LEFT_FRONT_INDEX]
        self.front_dist = msg.ranges[ROBOT_DIRECTION_FRONT_INDEX]
        self.rightfront_dist = msg.ranges[ROBOT_DIRECTION_RIGHT_FRONT_INDEX]
        self.right_dist = msg.ranges[ROBOT_DIRECTION_RIGHT_INDEX]
        self.rear_dist = msg.ranges[ROBOT_DIRECTION_REAR_INDEX]
        self.distances = msg.ranges

        #print("left: %.2f m\n" %self.left_dist,
        #      "left front: %.2f m\n" %self.leftfront_dist,
        #      "front: %.2f m\n" %self.front_dist,
        #      "right front: %.2f m\n" %self.rightfront_dist,
        #      "r: %.2f m\n" %self.right_dist,
        #      "rear: %.2f m\n" %self.rear_dist,
        #      "\n")
    
#.............................................................................................................


#Main.........................................................................................................
def main(args=None):
    rclpy.init(args=args)
    wallfollower = WallFollower()

    rclpy.spin(wallfollower)

    wallfollower.destroy_node
    rclpy.shutdown
#.............................................................................................................