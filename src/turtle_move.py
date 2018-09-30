#!/usr/bin/env python

import rospy
from math import atan2, radians, pow, sqrt
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

turtle_pose = Pose(0,0,0,0,0)

def poseCallback(pose_message):
    global turtle_pose
    turtle_pose = pose_message

def getDistance(x1,y1,x2,y2):
    return sqrt(pow((x1-x2),2) + pow((y1-y2),2))

def move(speed, distance, isForward):
    speed_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('turtle_move', anonymous=True)
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
    rospy.init_node('turtle_move', anonymous=True)
    rate = rospy.Rate(10)
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

def set_abs_rotation(desired_angle):
    global turtle_pose
    relative_angle = radians(desired_angle) - turtle_pose.theta
    isClockwise = relative_angle < 0
    rotate(abs(relative_angle)/4, abs(relative_angle), isClockwise)

def move_to_goal(goal, tolerance):
    global turtle_pose
    speed_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('turtle_move', anonymous=True)
    rate = rospy.Rate(10)
    twist = Twist()

    while getDistance(turtle_pose.x, turtle_pose.y, goal.x, goal.y) > tolerance:
        twist.linear.x = 1.5*getDistance(turtle_pose.x, turtle_pose.y, goal.x, goal.y)
        twist.linear.y = 0
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 4*(atan2(goal.y-turtle_pose.y, goal.x-turtle_pose.x)-turtle_pose.theta)

        speed_pub.publish(twist)
        rate.sleep()
    twist.linear.x = 0
    twist.angular.z = 0
    speed_pub.publish(twist)

def grid_clean():
    global turtle_pose
    speed_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('turtle_move', anonymous=True)
    rate = rospy.Rate(10)
    twist = Twist()
    goal = Pose(1,1,0,0,0)

    move_to_goal(goal, 0.01)
    set_abs_rotation(0)

    move(2, 9, 1)
    rotate(radians(10), radians(90), 0)

    i = 0
    while i < 5:
        i = i + 1
        move(3, 9, 1)
        rotate(radians(30), radians(90), 0)
        move(3, 1, 1)
        rotate(radians(30), radians(90), 0)
        move(3, 9, 1)
        rotate(radians(30), radians(90), 1)
        move(3, 1, 1)
        rotate(radians(30), radians(90), 1)
        move(3, 9, 1)

def spiral_clean():
    global turtle_pose
    speed_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('turtle_move', anonymous=True)
    rate = rospy.Rate(10)
    twist = Twist()
    count = 0
    const_speed = 4
    vk = 1
    wk = 2
    rk = 0.5

    while turtle_pose.x < 10.5 and turtle_pose.y < 10.5:
        rk = rk + 0.1
        twist.linear.x = rk
        twist.linear.y = 0
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = const_speed #vk / (0.1 + rk)
        speed_pub.publish(twist)
        rate.sleep()
    twist.linear.x = 0
    speed_pub.publish(twist)

if __name__ == '__main__':
    try:
        rospy.init_node('turtle_move', anonymous=True)    
        rospy.Subscriber('/turtle1/pose', Pose, poseCallback)

        menu = input('1 - Spiral cleaning\n2 - Grid Cleaning\nEnter: ')
        if menu == 1:
            spiral_clean()
        elif menu == 2:
            grid_clean()
        else:
            print usage()
            sys.exit(1)
    except rospy.ROSInterruptException:
        pass
