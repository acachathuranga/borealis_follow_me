import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
import copy
import numpy as np
import sys
import math
from threading import Lock, Timer

class CheckObstacles():

    def __init__(self):
        """ Calculates obstacle avoidance steering control, based on 3 Sonar sensors
        """

        sonar_left_topic = rospy.get_param(rospy.get_name()+'/sonar_left')                              #, '/uav1/sonar/sonar_left'
        sonar_mid_topic = rospy.get_param(rospy.get_name()+'/sonar_mid')                                #, '/uav1/sonar/sonar_mid'
        sonar_right_topic = rospy.get_param(rospy.get_name()+'/sonar_right')                            #, '/uav1/sonar/sonar_right'
        self.obstacle_range = rospy.get_param(rospy.get_name()+'/obstacle_range')                       #, 0.5
        self.avoidance_gain = rospy.get_param(rospy.get_name()+'/avoidance_gain')                       #, 0.3

        # Initializing to a long range
        self.sonar_left = self.obstacle_range * 2
        self.sonar_mid = self.obstacle_range * 2
        self.sonar_right = self.obstacle_range * 2

        self.sonar_left_time_stamp = rospy.get_time()
        self.sonar_mid_time_stamp = rospy.get_time()
        self.sonar_right_time_stamp = rospy.get_time()

        # self.sonar_left_sub = rospy.Subscriber(sonar_left_topic, Float32, self.sonar_left_cb)
        # self.sonar_mid_sub = rospy.Subscriber(sonar_mid_topic, Float32, self.sonar_mid_cb)
        # self.sonar_right_sub = rospy.Subscriber(sonar_right_topic, Float32, self.sonar_right_cb)

        self.sonar_left_sub = rospy.Subscriber(sonar_left_topic, Range, self.sonar_left_cb)
        self.sonar_mid_sub = rospy.Subscriber(sonar_mid_topic, Range, self.sonar_mid_cb)
        self.sonar_right_sub = rospy.Subscriber(sonar_right_topic, Range, self.sonar_right_cb)

        # Sensor Status monitoring
        self.sensor_monitoring_interval = 1
        self.timer = ClassTimer(self.sensor_monitor, self.sensor_monitoring_interval)

        self.threading_lock = Lock()

    def sonar_left_cb(self, range):
        self.threading_lock.acquire()
        self.sonar_left = range.range
        # self.sonar_left = range.data / 100.0
        self.sonar_left_time_stamp = rospy.get_time()
        self.threading_lock.release()

    def sonar_mid_cb(self, range):
        self.threading_lock.acquire()
        self.sonar_mid = range.range
        # self.sonar_mid = range.data / 100.0
        self.sonar_mid_time_stamp = rospy.get_time()
        self.threading_lock.release()

    def sonar_right_cb(self, range):
        self.threading_lock.acquire()
        self.sonar_right = range.range
        # self.sonar_right = range.data / 100.0
        self.sonar_right_time_stamp = rospy.get_time()
        self.threading_lock.release()

    def terminate(self):
        self.sonar_left_sub.unregister()
        self.sonar_mid_sub.unregister()
        self.sonar_right_sub.unregister()
        self.timer.stop()
        sys.exit()
        
    def caculate_steering(self):
        """
            Calculates steering feedback to avoid obstacles

            Returns required steering and linear vel attenuation
        """
        self.threading_lock.acquire()
        left = self.sonar_left
        mid = self.sonar_mid
        right = self.sonar_right
        self.threading_lock.release()
        
        if left > self.obstacle_range: left = 1
        if mid > self.obstacle_range: mid = 1
        if right > self.obstacle_range: right = 1

        # Ignoring 0 values (Due to sensor reading out of range)
        if left == 0: left = 1
        if mid == 0: mid = 1
        if right == 0: right = 1

        # Dynamic repulsion
        feedback = self.avoidance_gain * ( (-1.0/left) + (1.0/right) )
        # Scaling feedback
        feedback = feedback / mid

        # Linear Velocity attenuation
        if mid < 0.15:
            linear_vel_attenuation = mid

        return feedback, mid

    def avoid_obstacles(self, linear_vel, angular_vel):
        """
            Modifies a given command velocity, to avoid obstacles

            Returns state, linear, angular velocities

            if state == False, the goal is unreachable
        """
        self.threading_lock.acquire()
        left = self.sonar_left
        mid = self.sonar_mid
        right = self.sonar_right
        self.threading_lock.release()
        
        if left > self.obstacle_range: left = 1
        if mid > self.obstacle_range: mid = 1
        if right > self.obstacle_range: right = 1

        # Ignoring 0 values (Due to sensor reading out of range)
        if left == 0: left = 1
        if mid == 0: mid = 1
        if right == 0: right = 1

        # Dynamic repulsion
        feedback = self.avoidance_gain * ( (-1.0/left) + (1.0/right) )
        # Scaling feedback
        feedback = feedback / mid
        
        state = False
        if (mid < (self.obstacle_range / 2)) or (math.fabs(feedback) > 1):
            linear_vel = 0
            angular_vel += feedback
        else:
            angular_vel += feedback
            state = True
        # print (feedback, mid)
        return state, linear_vel, angular_vel

    def sensor_monitor(self):
        time = rospy.get_time()
        if (time - self.sonar_left_time_stamp) > self.sensor_monitoring_interval:
            rospy.logerr("Left Sonar sensor Not working!") 
        
        if (time - self.sonar_mid_time_stamp) > self.sensor_monitoring_interval:
            rospy.logerr("Mid Sonar sensor Not working!") 

        if (time - self.sonar_right_time_stamp) > self.sensor_monitoring_interval:
            rospy.logerr("Right Sonar sensor Not working!") 

class ClassTimer():
    def __init__(self, callback, interval):
        self.callback = callback
        self.interval = interval
        self.timer = Timer(self.interval, self.cb)
        self.timer.start()

    def cb(self):
        self.timer = Timer(self.interval, self.cb)
        self.timer.start()
        self.callback()

    def stop(self, *args):
        self.timer.cancel()
        
