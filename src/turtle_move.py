#!/usr/bin/env python

import sys
import rospy
from math import radians
from geometry_msgs.msg import Twist

def move(speed, distance, isForward):
    speed_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    twist = Twist()

    if isForward: twist.linear.x = abs(speed)
    else: twist.linear.x = -abs(speed)
    twist.linear.y = 0
    twist.linear.z = 0

    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    t0 = rospy.get_time()
    curr_distance = 0
    while curr_distance < distance:
        t1 = rospy.get_time()
        curr_distance = speed * (t1-t0)
        speed_pub.publish(twist)
        rate.sleep()

    twist.linear.x = 0
    speed_pub.publish(twist)

def rotate(angular_speed, relative_angle, isClockwise):
    speed_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0

    twist.angular.x = 0
    twist.angular.y = 0
    if isClockwise: twist.angular.z = -abs(angular_speed)
    else: twist.angular.z = abs(angular_speed)

    t0 = rospy.get_time()
    curr_ang = 0.0
    while curr_ang < relative_angle:
        t1 = rospy.get_time()
        curr_ang = angular_speed * (t1-t0)
        speed_pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0
    speed_pub.publish(twist)

def usage():
    return '%s [speed distance isForward]'%sys.argv[0]

if __name__ == '__main__':
    if len(sys.argv) == 4:
        speed = float(sys.argv[1])
        distance = float(sys.argv[2])
        isForward = int(sys.argv[3])
    else:
        print usage()
        sys.exit(1)
    try:
        move(speed, distance, isForward)
        rotate(radians(10), radians(90), 1)
    except rospy.ROSInterruptException:
        pass