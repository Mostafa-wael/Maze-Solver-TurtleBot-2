#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time


class Robot:

    def __init__(self):
        rospy.init_node('robot_control_node')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_subscriber = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.laserCallback, callback_args=None, queue_size=1)
        self.command = Twist()
        self.laserMsg = LaserScan()
        self.ctrl_c = False
        self.rate = rospy.Rate(0.5)
        # using this function allows us to do some operations before actually shutting down
        rospy.on_shutdown(self.shutdownHook)

    def publishCommand(self):
        while not self.ctrl_c:
            connections = self.pub.get_num_connections()  # get the number of connections to other ROS nodes for this topic
            if connections > 0:
                self.pub.publish(self.command)
                break
            else:
                self.rate.sleep()

    def shutdownHook(self):
        self.ctrl_c = True

    def laserCallback(self, msg):
        self.laserMsg = msg

    def getLaserAt(self, pos):
        time.sleep(1)
        return self.laserMsg.ranges[pos]

    def stopRobot(self):
        self.command.linear.x = 0.0
        self.command.angular.z = 0.0
        self.publishCommand()

    def moveStraight(self):
        # Initialize velocities
        self.command.linear.x = 0.5
        self.command.linear.y = 0
        self.command.linear.z = 0
        self.command.angular.x = 0
        self.command.angular.y = 0
        self.command.angular.z = 0

        # Publish the velocity
        self.publishCommand()

    def turn(self, clockwise, speed, time):

        # Initilize velocities
        self.command.linear.x = 0
        self.command.linear.y = 0
        self.command.linear.z = 0
        self.command.angular.x = 0
        self.command.angular.y = 0
        
        if clockwise:
            self.command.angular.z = -speed
        else:
            self.command.angular.z = speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.pub.publish(self.command)
            i += 1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stopRobot()


class mazeSolver:

    def __init__(self):
        print("I'm initializing...")
        self.RB = Robot()
        self.laserFront = self.RB.getLaserAt(360)
        self.clockwise = True
        # turn function takes 3 parameters: direction, speed (rad/sec), time (sec)
        # to turn the robot by a specified angle, send the angle as the speed set the time to 1
        self.turn_speed = 0.785 # at time equals 1, it is equivalent to 45 degrees
        self.turn_time = 1

    def move(self):
        while self.laserFront > 1:
            self.RB.moveStraight()
            self.laserFront = self.RB.getLaserAt(360)
            print("moving forward, Distance: ", self.laserFront)
        self.RB.stopRobot()

    def turn(self):
        northEast = self.RB.getLaserAt(180)
        northWest = self.RB.getLaserAt(540)
        print("northEast: ", northEast, "northWest", northWest)
        if northEast >= northWest:
            while self.laserFront < 1:
                self.RB.turn(self.clockwise, self.turn_speed, self.turn_time)
                self.laserFront = self.RB.getLaserAt(360)
                print("point to northEast")
        else:
            while self.laserFront < 1:
                self.RB.turn((not self.clockwise), self.turn_speed, self.turn_time)
                self.laserFront = self.RB.getLaserAt(360)
                print("point to northWest")
        self.RB.stopRobot()
    def solveMaze(self):
        while not self.RB.ctrl_c:
            self.move()
            self.turn()


if __name__ == '__main__':

    robo = mazeSolver()
    robo.solveMaze()
