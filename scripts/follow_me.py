#!/usr/bin/python

import os
# Constrain OPENBLAS Multithreading (To solve Numpy Performance Issues)
os.environ['OPENBLAS_NUM_THREADS'] = '1'

import rospy
import sys
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist
from std_srvs.srv._SetBool import SetBool, SetBoolRequest, SetBoolResponse
from threading import Timer
from controller import RobotController
import signal

human_pose_sub = None
human_path_sub = None
uav_pose_sub = None
cmd_vel_pub = None
robot_controller = None
autonomous_mode = True

def human_pose_cb(pose):
    robot_controller.updateLeaderPose(pose)

def human_path_cb(path):
    pass

def uav_pose_cb(pose):
    robot_controller.updatePose(pose)

def publish_velocity(cmd_vel):
    if autonomous_mode:
        cmd_vel_pub.publish(cmd_vel)

def set_mode(req):
    global autonomous_mode
    if (req.data == True):
        autonomous_mode = True
        msg = "Autonomous Mode ON"
    else:
        autonomous_mode = False
        msg = "Autonomous Mode OFF"
    print (rospy.get_name() + ": " + msg)
    return SetBoolResponse(True, msg)

########### DEBUGGING #########
# def leader_pose_cb(odom):
#     pose = PoseWithCovarianceStamped()

#     pose.header = odom.header
#     pose.pose.pose =  odom.pose.pose
#     robot_controller.updateLeaderPose(pose)

# def self_pose_cb(odom):
#     pose = PoseWithCovarianceStamped()

#     pose.header = odom.header
#     pose.pose.pose =  odom.pose.pose
#     robot_controller.updatePose(pose)
###############################
    
def follow_me():
    
    rospy.init_node('follow_me')
    signal.signal(signal.SIGINT, terminate)

    human_pose_topic = rospy.get_param(rospy.get_name()+'/human_pose_topic')        #, '/HumanPose'
    human_path_topic = rospy.get_param(rospy.get_name()+'/human_pose_topic')        #, '/HumanPath'

    uav_pose_topic = rospy.get_param(rospy.get_name()+'/uav_pose_topic')            #, '/UAV1Pose'
    cmd_vel_topic = rospy.get_param(rospy.get_name()+'/cmd_vel_topic')              #, '/uav1/mobile_base/commands/velocity'

    activation_service = rospy.get_param(rospy.get_name()+'/activation_service')    #, '/uav1/follow_me/enable'

    follow_distance = rospy.get_param(rospy.get_name()+'/follow_distance')          #, 2

    ##### DEBUGGING ######
    # leader_pose_sub = rospy.Subscriber('/uav2/odom', Odometry, leader_pose_cb)
    # leader_pose_sub = rospy.Subscriber('/uav1/odom', Odometry, self_pose_cb)
    ######################

    # Logging Parameters
    print ("")

    global human_pose_sub, human_path_sub, uav_pose_sub, cmd_vel_pub, robot_controller
    robot_controller = RobotController(follow_distance, publish_velocity, obstacle_avoidance=True)
    
    human_pose_sub = rospy.Subscriber(human_pose_topic, PoseWithCovarianceStamped, human_pose_cb)
    human_path_sub = rospy.Subscriber(human_path_topic, Path, human_path_cb)
    uav_pose_sub = rospy.Subscriber(uav_pose_topic, PoseWithCovarianceStamped, uav_pose_cb)
    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1000)

    activation_srv = rospy.Service(activation_service, SetBool, set_mode)
    
    print ("UAV Follower Controller: %s" %(uav_pose_topic))
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def terminate(*args):
    print "Follow Me: User termination requested"
    robot_controller.stop()
    human_pose_sub.unregister()
    human_path_sub.unregister()
    uav_pose_sub.unregister()
    sys.exit()


if __name__ == '__main__':
    follow_me()




    

