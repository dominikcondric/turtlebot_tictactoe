# Subscriber to /goal_pose topic -> converts to data sent to cmd_vel topic for movement

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Odometry
from math import atan2, pi
import numpy as np
#from tf.transformations import euler_from_quaternion
from enum import Enum
import datetime

import transforms3d as t3d


class MovementStatus(Enum):
    MOVING = 1
    FINISHED = 2


class NavigateToPose(Node):
    def __init__(self):
        super().__init__('navigate_to_pose')
        self.go_to_position_tictactoe_subscriber = self.create_subscription(
            Point,
            '/go_to_position_tictactoe',
            self.go_to_position_tictactoe_callback,
            1
        )

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.currentOdom = self.create_subscription(
            Odometry,
            '/odom',
            self.calculate_movement_to_goal_pose,
            1
        )

        self.home = Point()
        self.home.x = 0.0
        self.home.y = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.isHome = True
        self.movement_status = MovementStatus.FINISHED
        self.linear_x = 0.
        self.angular_z = 0.


    def go_to_position_tictactoe_callback(self, msg):
        #ros2 topic pub /go_to_position_tictactoe geometry_msgs/Point "{x: 0, y: 1}"
        x = msg.x
        y = msg.y

        # Testirano na empty world launch iz gazeba pa zato te coord

        #        + (x)
        #        |
        # + < -- | --  > - (y)
        #        |
        #        -
        # (robot postavljen na shit način - gleda prema xpos, desno je yneg)

        #      |      |
        #  3,1 | 3,0  | 3,-1  <- 0,0 - 1,0 - 2,0 (x,y)
        # -----|------|-------
        #  2,1 | 2,0  | 2,-1  <- 0,1 - 1,1 - 2,1
        # -----|------|-------
        #  1,1 | 1,0  | 1,-1  <- 0,2 - 1,2 - 2,2
        #      |      |
        # ~~~~~~~0,0~~~~~~~~~ <- inital robot pos 


        # mapiranje:
        if (x == 0 and y == 0):
            self.goal_x = 3.0 #1.5
            self.goal_y = 1.0 #0.5
        elif (x == 1 and y == 0):
            self.goal_x = 3 #1.5
            self.goal_y = 0.0
        elif (x == 2 and y == 0):
            self.goal_x = 3.0 #1.5
            self.goal_y = -1.0 #-0.5
        elif (x == 0 and y == 1):
            self.goal_x = 2.0 #1.0
            self.goal_y = 1.0 #0.5
        elif (x == 1 and y == 1):
            self.goal_x = 2.0 #1.0
            self.goal_y = 0.0
        elif (x == 2 and y == 1):
            self.goal_x = 2.0 #1.0
            self.goal_y = -1 #-0.5
        elif (x == 0 and y == 2):
            self.goal_x = 1.0 #0.5
            self.goal_y = 1.0 #0.5
        elif (x == 1 and y == 2):
            self.goal_x = 1.0 #0.5
            self.goal_y = 0.0
        elif (x == 2 and y == 2):
            self.goal_x = 1.0 #0.5
            self.goal_y = -1.0 #-0.5
        elif (x == -1 and y == -1):
          #home 0,0
          print(f'Going home')
          self.goal_x = self.home.x
          self.goal_y = self.home.y

        print(f'Going to x: {self.goal_x}, y: {self.goal_y}')
        print(f'Mapped from tictactoe coods x: {x}, y: {y}')

    def calculate_movement_to_goal_pose(self, msg):
        linear_x, angular_z = self.calculate_movement(msg)
        if linear_x != self.linear_x or angular_z != self.angular_z:
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self.publisher.publish(msg)
        self.linear_x = linear_x
        self.angular_z = angular_z


    def calculate_movement(self, msg):
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y

        rotations = msg.pose.pose.orientation

        linear_x = 0.0
        angular_z = 0.0

        # istrazeno vise izvora na internetu, preuzet kod za funkciju pretvorbe iz quaterniona u euler, samo za yaw jer drugo nije potrebno:
        yaw = euler_from_quaternion(rotations.x, rotations.y, rotations.z, rotations.w)

        # postavljanje razmaka izmedu pozicije i cilja
        gap_x = abs(self.goal_x - pos_x)
        gap_y = abs(self.goal_y - pos_y)

        goal_angle = (np.arctan2(self.goal_y - pos_y, self.goal_x - pos_x)) - yaw

        if abs(goal_angle) > pi:
            goal_angle = goal_angle - 2 * pi
            if goal_angle < -(2 * pi):
                goal_angle = goal_angle + 4 * pi

        if gap_x > 0.1 or gap_y > 0.1:
            if (self.movement_status != MovementStatus.MOVING):
                self.movement_status = MovementStatus.MOVING
                print(self.movement_status)
                self.isHome = False

            if goal_angle > 0.3:
                linear_x = 0.0
                angular_z = 0.4
            elif goal_angle < -0.3:
                linear_x = 0.0
                angular_z = -0.4
            else:
                if (self.goal_x == -1.0 and self.goal_y == 0.0):
                    linear_x = 0.0
                else:
                    linear_x = 0.4
                angular_z = 0.0

        else:
            linear_x = 0.0
            angular_z = 0.0
            if (self.movement_status != MovementStatus.FINISHED):
                self.movement_status = MovementStatus.FINISHED
                print(self.movement_status)
                if (self.goal_x == self.home.x and self.goal_y == self.home.y):
                    self.isHome = True
                    self.face_laptop()
                if (not self.isHome):
                    #TODO: Spin and go home!
                    print(f'Going home')
                    self.goal_x = self.home.x
                    self.goal_y = self.home.y

        return linear_x, angular_z

    def face_laptop(self):
        self.goal_x = -1.0
        self.goal_y = 0.0

def euler_from_quaternion(x, y, z, w):
    """
    Izracun eulerovog kuta iz quaterniona.
    Parameters
    ----------
    x : dobiven iz rotacije robota, msg.pose.pose.orientation.x
    y : dobiven iz rotacije robota, msg.pose.pose.orientation.y
    z : dobiven iz rotacije robota, msg.pose.pose.orientation.z
    w : dobiven iz rotacije robota, msg.pose.pose.orientation.w
    Returns
    ----------
        yaw in radians
    """

    t3= 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return yaw_z

def main(args=None):
    rclpy.init(args=args)

    navigate_to_pose = NavigateToPose()

    rclpy.spin(navigate_to_pose)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
