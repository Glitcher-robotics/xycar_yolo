#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import LaserScan
from darknet_ros_msgs.msg import BoundingBoxes
from xycar_motor.msg import xycar_motor #ready for lidar, yolo, motor

import signal
import sys
import os

def signal_handler(sig, frame): #program down with ctrl+C
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

steering_pub = None
box_data = []
lidar_data = []

motor_msg = xycar_motor() #ready for motor
person = None

def init_node():
    global steering_pub
    rospy.init_node('human_track')
    rospy.Subscriber('/scan', LaserScan, callback_lidar) # topic Subscriber
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)
    steering_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

def exit_node():
    print('finished')

def drive_back(): #detect object & drive back
    for back_cnt in range(10):
        drive(50, -2)
        time.sleep(0.1)


def search_person(): #search person & rotate
    for back_cnt in range(10):
        drive(-50, 2)
        time.sleep(0.1)

def drive(angle, speed): #motor control def
    global steering_pub
    motor_msg.header.stamp = rospy.Time.now()
    motor_msg.angle = angle
    motor_msg.speed = speed
    steering_pub.publish(motor_msg)

def callback(msg): #yolo class detect
    global box_data
    box_data = msg

def callback_lidar(data):
    global lidar_data
    lidar_data = data.ranges

def detect_lidar(): # collision detect with lidar
    global lidar_data
    ok=0

    if len(lidar_data) != 720:
        return True
    
    for degree in range(0, 180):
        if lidar_data[degree] < 0.3:
            ok += 1
        if ok > 3:
            return True
    return False

if __name__=='__main__':
    init_node()
    No_person = 1000000

    person_position = 0
    time.sleep(15)

    while not rospy.is_shutdown():
        person_position = No_person
        boxes = box_data

        if not boxes:
            continue

        for i in range(len(boxes.bounding_boxes)):
            if boxes.bounding_boxes[i].Class == 'person':
                person_position = int((((boxes.bounding_boxes[i].xmas + boxes.bounding_boxes[i].xmin)/2.0)-320.0)/320.0*50.0)
                print "person position: ", person_position

            if person_position == No_person :
                if detect_lidar(): # no person, if detect object, drive back, else search person
                    drive_back()
                else:
                    search_person()

            else:
                angle = person_position

                if detect_lidar():
                    drive(angle, 0)
                else:
                    drive(angle, 2)

            time.sleep(0.1)

        rospy.on_shutdown(exit_node)