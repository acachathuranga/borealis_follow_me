import rospy
from sensor_msgs.msg import Range
import copy
import numpy as np
import sys
import math
from threading import Lock

class CheckObstacles():

    def __init__(self):
        """ Calculates obstacle avoidance steering control, based on 3 Sonar sensors
        """

        self.sonar_left = None
        self.sonar_mid = None
        self.sonar_right = None
        
        sonar_left_topic = rospy.get_param(rospy.get_name()+'/sonar_left', '/uav1/sonar/sonar_left')
        sonar_mid_topic = rospy.get_param(rospy.get_name()+'/sonar_mid', '/uav1/sonar/sonar_mid')
        sonar_right_topic = rospy.get_param(rospy.get_name()+'/sonar_right', '/uav1/sonar/sonar_right')
        self.obstacle_range = rospy.get_param(rospy.get_name()+'/obstacle_range', 0.5)
        self.avoidance_gain = rospy.get_param(rospy.get_name()+'/avoidance_gain', 0.3)

        self.sonar_left_sub = rospy.Subscriber(sonar_left_topic, Range, self.sonar_left_cb)
        self.sonar_mid_sub = rospy.Subscriber(sonar_mid_topic, Range, self.sonar_mid_cb)
        self.sonar_right_sub = rospy.Subscriber(sonar_right_topic, Range, self.sonar_right_cb)

        self.threading_lock = Lock()

    def sonar_left_cb(self, range):
        self.threading_lock.acquire()
        self.sonar_left = range.range
        self.threading_lock.release()

    def sonar_mid_cb(self, range):
        self.threading_lock.acquire()
        self.sonar_mid = range.range
        self.threading_lock.release()

    def sonar_right_cb(self, range):
        self.threading_lock.acquire()
        self.sonar_right = range.range
        self.threading_lock.release()

    def terminate(self):
        self.sonar_left_sub.unregister()
        self.sonar_mid_sub.unregister()
        self.sonar_right_sub.unregister()
        sys.exit()
        
    def caculate_steering(self):
        """
            Calculates steering feedback to avoid obstacles
        """
        self.threading_lock.acquire()
        left = self.sonar_left
        mid = self.sonar_mid
        right = self.sonar_right
        self.threading_lock.release()

        if left > self.obstacle_range: left = 1
        if mid > self.obstacle_range: mid = 1
        if right > self.obstacle_range: right = 1

        feedback = self.avoidance_gain * ( (-1.0/left) + (1.0/right) )
        # Scaling feedback
        feedback = feedback / mid

        return feedback

    
        