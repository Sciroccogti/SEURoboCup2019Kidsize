#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import cv2
import numpy
from game_ctrl.srv import TeamInfo
from simulation.srv import ImageInfo
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from simulation.msg import Task
from simulation.msg import GameData
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from multiprocessing import Lock

robot_name = 'robot'
image_width = 640 
image_height = 480 
image_channels = 4
image_data = numpy.array([])
my_position = Point()
gamedata = GameData()
image_mutex = Lock()
pos_mutex = Lock()
gdata_mutex = Lock()
imagePublisher = None
taskPublisher = None
headPublisher = None
M_PI = 3.14159265358979323846

def GetRobot(tname=''):
    rospy.wait_for_service('/teaminfo')
    try:
        rbt = rospy.ServiceProxy('/teaminfo', TeamInfo)
        res = rbt(tname)
        return res.team
    except rospy.ServiceException, e:
        # rospy.WARN("Service /teaminfo call failed: %s", e)
        rospy.logwarn("Service /teaminfo call failed: %s", e)
        return ""

def GetImageInfo():
    rospy.wait_for_service(robot_name+'/imageinfo')
    try:
        info = rospy.ServiceProxy(robot_name+'/imageinfo', ImageInfo)
        res = info()
        global image_width
        global image_height
        global image_channels
        image_width = res.width
        image_height = res.height
        image_channels = res.channels
    except rospy.ServiceException, e:
        # rospy.WARN("Service /imageinfo call failed: %s", e)
        rospy.logwarn("Service /teaminfo call failed: %s", e)


def ImageUpdate(image):
    global image_data
    global image_mutex
    image_mutex.acquire()
    np_arr = numpy.fromstring(image.data, numpy.uint8)
    image_data = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # image_data = CvBridge().imgmsg_to_cv2(image)#, 'bgr8')
    # print("image_data updated")
    image_mutex.release()

def PositionUpdate(pos):
    global my_position
    global pos_mutex
    pos_mutex.acquire()
    my_position=pos
    pos_mutex.release()

def GdataUpdate(gdata):
    global gamedata
    global gdata_mutex
    gdata_mutex.acquire()
    gamedata=gdata
    gdata_mutex.release()

def ImagePublish(image, encoding='bgr8'):
    imagePublisher.publish(CvBridge().cv2_to_imgmsg(image, encoding))

def TaskPublish(task):
    taskPublisher.publish(task)

def HeadPublish(yaw, pitch):
    p = Point()
    p.z = yaw
    p.y = pitch
    headPublisher.publish(p)

if __name__ == '__main__':
    teamname = ''
    hostname = 'localhost'
    if len(sys.argv)<2:
        print "no teamname set"
        exit(0)
    teamname = sys.argv[1]
    if len(sys.argv)>2:
        with open(sys.argv[2], 'r') as f:
            hostname = f.readline()
            hostname.strip()
    os.environ['ROS_MASTER_URI'] = 'http://{}:11311'.format(hostname)
    rospy.init_node(teamname, anonymous=True)

    robot_name = GetRobot(teamname)
    if len(robot_name)==0:
        # rospy.WARN("no robot found")
        rospy.logwarn("no robot found")

        exit(0)
    GetImageInfo()
    
    imageSubscriber = rospy.Subscriber(robot_name+'/image/compressed', CompressedImage, ImageUpdate)
    posSubscriber = rospy.Subscriber(robot_name+'/position', Point, PositionUpdate)
    gdataSubscriber = rospy.Subscriber("/gamedata", GameData, GdataUpdate)
    imagePublisher = rospy.Publisher(teamname+"/image", Image, queue_size=1)
    taskPublisher = rospy.Publisher(robot_name+"/task", Task, queue_size=1)
    headPublisher = rospy.Publisher(robot_name+"/head", Point, queue_size=1)

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        image_mutex.acquire()
        image_tmp = image_data.copy()
        image_mutex.release()
        gdata_mutex.acquire()
        gdata_tmp = gamedata
        gdata_mutex.release()
        pos_mutex.acquire()
        pos_tmp = my_position
        pos_mutex.release()
        
        if gdata_tmp.state == GameData.STATE_PLAY:
            task = Task()
            task.type = Task.TASK_WALK
            task.step = 0.03
            head_yaw = - M_PI / 2
            head_pitch = 0.0
            if(image_tmp != []):
                gray = None
                cv2.cvtColor(image_tmp, cv2.COLOR_BGR2GRAY, gray)

            HeadPublish(head_yaw, head_pitch)
            TaskPublish(task)

        rate.sleep()


