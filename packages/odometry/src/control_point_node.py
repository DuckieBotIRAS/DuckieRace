#!/usr/bin/env python3

from enum import Enum
import rospy
import tf   
import numpy as np
from std_msgs.msg import Float64,UInt8
from geometry_msgs.msg import Point
from enum import Enum
from nav_msgs.msg import Odometry
#from followlane.msg import MyOdom
from std_msgs.msg import Float64
import math

from my_msg.msg import OdomPosition
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
import os
from duckietown.dtros import DTROS, NodeType
import math

class LaneDetected(Enum):
    BOTH_LINES = 0
    WHITE_LINE = 1
    YELLOW_LINE = 2
    NO_LINE = 3

class ControlPointNode(DTROS):
    def __init__(self,node_name):
        super(ControlPointNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        

        vehicle_name = os.environ['VEHICLE_NAME']
        
        point_topic = f"/{vehicle_name}/drive/point"
        self.sub_point = rospy.Subscriber(point_topic, Point, self.cbDriveToPoint, queue_size = 1)

        forward_topic = f"/{vehicle_name}/drive/forward"
        self.sub_forward = rospy.Subscriber(forward_topic, Float64, self.cbForward, queue_size = 1)

        turn_topic = f"/{vehicle_name}/drive/turn"
        self.sub_turn = rospy.Subscriber(turn_topic, Float64, self.cbTurn, queue_size = 1)

        #self.sub_pos = rospy.Subscriber(odom_topic, my_odom, self.cbUpdatePosition, queue_size = 1)
        #self.sub_pos_x =     rospy.Subscriber(odom_topic + 'X'      , Float64, self.cbUpdatePositionX, queue_size = 1)
        #self.sub_pos_y =     rospy.Subscriber(odom_topic + 'Y'      , Float64, self.cbUpdatePositionY, queue_size = 1)
        #self.sub_pos_theta = rospy.Subscriber(odom_topic + 'Theta'  , Float64, self.cbUpdatePositionTheta, queue_size = 1)

        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self.pub_wheel_vel = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        twist_topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size = 1)

        finish_topic = f"/{vehicle_name}/drive/finished"
        self.pub_finish = rospy.Publisher(finish_topic, UInt8, queue_size = 1)

        odom_topic = f"/{vehicle_name}/odom/position"        
        self.sub_pos = rospy.Subscriber(odom_topic, OdomPosition, self.cbUpdatePosition, queue_size = 1)
        

        
        self.lastError_v = 0
        self.lastError_a = 0
        self.theta = 0
        self.x = 0
        self.y = 0

        self.MAX_VEL = 0.3

        self.is_driving = False

        rospy.on_shutdown(self.fnShutDown)

    #def cbUpdatePositionX(self,msg):
    #    self.x = msg.data
    #def cbUpdatePositionY(self,msg):
    #    self.y = msg.data
    #def cbUpdatePositionTheta(self,msg):
    #    self.theta = msg.data
    
    def cbUpdatePosition(self, msg):
        #(f'this worked new point at {(msg.x,msg.y)} : {msg.theta}')
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def cbTurn(self, msg):
        print(f'received turn message {msg}')

        if not self.is_driving:
            self.is_driving = True
            self.turn((self.theta + msg.data)%360)
    
    def cbForward(self,msg):
        print(f'received forward message {msg}')
        #TODO don't this works if it roateted before?
        x,y = rotate(msg.data,0,self.theta)

        point = lambda : None
        point.x = self.x +x
        point.y = self.y +y
        print(f'going to drive from {(self.x,self.y)} to {(point.x,point.y)}')

        if not self.is_driving:
            self.is_driving = True
            self.DriveToPoint(point)
    

    def cbDriveToPoint(self, destination_point):
        print(f'received message {destination_point}')
        x,y = rotate(destination_point.x,destination_point.y,self.theta)

        destination_point.x = self.x + x
        destination_point.y = self.y + y
        print(f'going to drive from {(self.x,self.y)} to {(destination_point.x,destination_point.y)}')

        if not self.is_driving:
            self.is_driving = True
            self.DriveToPoint(destination_point)

    def turn(self,theta_dest):
        
        delta_theta = theta_dest - self.theta
        delta_theta = (delta_theta + 180) % 360 -180
        error_a = delta_theta


        print(f'currently facing {self.theta}. Destination {theta_dest}. Turning : {error_a}')
        rate = rospy.Rate(30)
        while (error_a > 5 or error_a < -5) and not rospy.is_shutdown():
            print(f'currently facing {self.theta}. Destination {theta_dest}. Turning : {error_a}')

            

            delta_theta = theta_dest - self.theta
            delta_theta = (delta_theta + 180) % 360 -180
            error_a = delta_theta

            Kp_a = 0.0125 * 5
            Kd_a = 0.035 * 5 

            print(f'currently facing : {self.theta} goal is at {theta_dest} using error of {error_a}\n')

            if error_a  > 0 :
                vel_left = -0.2
                vel_right = 0.2 
            else:
                vel_left = 0.2
                vel_right = -0.2 


            msg = WheelsCmdStamped(vel_left=vel_left, vel_right=vel_right)
            self.pub_wheel_vel.publish(msg)
            rate.sleep()
        
        print('done')
        msg = WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
        self.pub_wheel_vel.publish(msg)
        
                
        print(f'done currently facing {self.theta}. Destination {theta_dest}')
        self.pub_finish.publish(UInt8(1))
        self.is_driving = False

    

    def DriveToPoint(self, dest):
        error_v = np.sqrt((self.x - dest.x)**2 + (self.y -dest.y)**2)
        rate = rospy.Rate(10)

        while (error_v > 0.1) and not rospy.is_shutdown():
            print(f'current Position ({self.x}, {self.y}) : {self.theta} \n')
            error_v = np.sqrt((self.x - dest.x)**2 + (self.y -dest.y)**2)
            Kp_v = 0.125 * 5
            Kd_v = 0.35 * 5

                

            v = min(Kp_v * error_v + Kd_v * (error_v -self.lastError_v), self.MAX_VEL)

            print(f'distance : {error_v} useing speed {v} \n')

            # direction error
            scale = 1 / error_v
            theta_dest = calc_theta(dx = (self.x - dest.x), dy= (self.y -dest.y))

            Kp_a = 0.0125 * 5 #*5 
            Kd_a = 0.035 * 5 #*5

            #calculate error in angle: 
            delta_theta = theta_dest - self.theta
            delta_theta = (delta_theta + 180) % 360 -180
            error_a = delta_theta

            print(f'currently facing : {self.theta} goal is at {theta_dest} using error of {error_a}\n')

            a = max(min(Kp_a * error_a + Kd_a * (error_a - self.lastError_a) , 5),-5)

            self.lastError_a = error_a
            self.lastError_v = error_v
            
            #return
        
            twist = Twist2DStamped(v=v, omega=a)
            print(f'moving {v} {a} error {error_v} error angle {error_a}')
            self.pub_cmd_vel.publish(twist)

            rate.sleep()
        
        print('arrived')
        twist = Twist2DStamped(v=0.0, omega=0.0)
        #TODO reinsert for movement
        self.pub_cmd_vel.publish(twist) 
        self.pub_finish.publish(UInt8(1))
        self.is_driving = False


    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist) 

def calc_theta(dx,dy):
    error = np.sqrt(dx**2 + dy**2)
    scale = 1 / error
    if dx >= 0 and dy >= 0:
        return math.asin(dy*scale) / (np.pi *2) * 360 
    elif dx >= 0 and dy < 0:
        return 360 + math.asin(dy*scale) / (np.pi *2) * 360 
    elif dx < 0 and dy >= 0:
        return 180 - math.asin(dy*scale) / (np.pi *2) * 360 
    else:
        return 180 - math.asin(dy*scale) / (np.pi *2) * 360 

def rotate(x,y,theta):

    print(f'got ({x}:{y}) rotating {theta}')
    a = math.radians(theta)
    rx = x * math.cos(a) + y * math.sin(a)
    ry = -x*math.sin(a) + y * math.cos(a)

    return (-rx,ry)

if __name__ == '__main__':
    # create the node
    node = ControlPointNode(node_name='control_point_node')
    #node.cbDriveToPoint(Point(x=1,y=1,z=0))
    # keep the process from terminating
    rospy.spin()