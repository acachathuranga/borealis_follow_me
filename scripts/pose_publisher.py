#!/usr/bin/python

import os
# Constrain OPENBLAS Multithreading (To solve Numpy Performance Issues)
os.environ['OPENBLAS_NUM_THREADS'] = '1'

import rospy
import sys
from gazebo_msgs.msg import LinkState, LinkStates
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

###########################################
############## PARAMETERS #################
link_state_topic = '/gazebo/link_states'

human_tag = 'human::base_footprint'
uav1_tag = 'uav1::base_footprint'
uav2_tag = 'uav2::base_footprint'
uav3_tag = 'uav3::base_footprint'

human_pose_topic = 'HumanPose'
uav1_pose_topic = 'UAV1Pose'
uav2_pose_topic = 'UAV2Pose'
uav3_pose_topic = 'UAV3Pose'

frame_id = 'odom'

publish_rate = 25       # Hz
###########################################

human_pub = None
uav1_pub = None
uav2_pub = None
uav3_pub = None
last_msg_time = 0
seq = 0

def extractPose(link_states, tag):
    """
        Extracts the pose relevant to a given link name from Gazebo link states topic,
        and returns it as a poseWithCovarianceStamped message
    """
    index = link_states.name.index(tag)
    pose = link_states.pose[index]

    pose_stamped = PoseWithCovarianceStamped()
    pose_stamped.header.seq = seq
    pose_stamped.header.stamp = rospy.get_rostime()
    pose_stamped.header.frame_id = frame_id

    pose_stamped.pose.pose = pose

    return pose_stamped

def publishHumanPose(link_states):
    pass
    # human_pub.publish(extractPose(link_states, human_tag))

def publishUAV1Pose(link_states):
    uav1_pub.publish(extractPose(link_states, uav1_tag))

def publishUAV2Pose(link_states):
    uav2_pub.publish(extractPose(link_states, uav2_tag))

def publishUAV3Pose(link_states):
    uav3_pub.publish(extractPose(link_states, uav3_tag))

def callback(link_states):
    if (rospy.get_time() - last_msg_time) > (1.0 / publish_rate):
        publishHumanPose(link_states)
        publishUAV1Pose(link_states)
        publishUAV2Pose(link_states)
        publishUAV3Pose(link_states)

        global last_msg_time, seq
        last_msg_time = rospy.get_time()
        seq += 1

    
    
def pose_publisher():
    print ("Gazebo Pose Publisher")
    rospy.init_node('gazebo_pose_publisher')

    global human_pub, uav1_pub, uav2_pub, uav3_pub

    human_pub = rospy.Publisher(human_pose_topic, PoseWithCovarianceStamped, queue_size=100)
    uav1_pub = rospy.Publisher(uav1_pose_topic, PoseWithCovarianceStamped, queue_size=100)
    uav2_pub = rospy.Publisher(uav2_pose_topic, PoseWithCovarianceStamped, queue_size=100)
    uav3_pub = rospy.Publisher(uav3_pose_topic, PoseWithCovarianceStamped, queue_size=100)

    rospy.Subscriber(link_state_topic, LinkStates, callback)

    print ("Pose Publisher Running!")
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pose_publisher()

    

