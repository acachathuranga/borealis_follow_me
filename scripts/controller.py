import rospy
from threading import Timer, Lock
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion, Twist
from tf import Transformer, transformations
import copy
import numpy as np
from collections import deque
import sys
import math
from obstacle_avoidance import CheckObstacles

class RobotController():

    def __init__(self, distance, callback, rate=10, obstacle_avoidance=False):
        """ Given a leader path, pose and self pose, follows the leader at a specific distance

            :distance: Following distance to leader
            :callback: Callback function that accepts geometric_msgs/Twist msg (Command Velocity)
            :rate: (Optional) Controller rate
            :obstacle_avoidance: (Optional) Use obstacle avoidance module if True. 
        """
        
        self.publish_velocity = callback
        self.obstacle_avoidance = obstacle_avoidance
        self.distance = distance

        interval = 1.0 / rate       # Seconds
        # self.timer = ClassTimer(self.controller, interval)\
        self.timer = ClassTimer(self.strict_controller, interval)
        # self.timer = ClassTimer(self.circular_follower, interval)

        self.thread_lock = Lock()

        self.path = deque(maxlen=5000)      # x, y, theta, distance    Note: Distance 0 is the position of leader when the node is started
        self.pose = [0,0,0]                 # x, y, theta

        self.controller_last_timestamp = rospy.get_time()
        self.angular_Kp = rospy.get_param(rospy.get_name()+'/angular_Kp')                           #, 1.0
        self.angular_Kd = rospy.get_param(rospy.get_name()+'/angular_Kd')                           #, 0.0
        self.angular_Ki = rospy.get_param(rospy.get_name()+'/angular_Ki')                           #, 0.06
        self.max_angular_velocity = rospy.get_param(rospy.get_name()+'/max_angular_velocity')       #, 2.0
        self.linear_velocity_gain = rospy.get_param(rospy.get_name()+'/linear_velocity_gain')       #, 2.0
        self.max_linear_velocity = rospy.get_param(rospy.get_name()+'/max_linear_velocity')         #, 0.7
        self.distance_tolerance = rospy.get_param(rospy.get_name()+'/distance_tolerance')           #, 0.3
        self.obs_vel_attenuation = rospy.get_param(rospy.get_name()+'/obs_vel_attenuation')         #, 10
        self.linear_velocity_attenuation = rospy.get_param(rospy.get_name()+'/linear_velocity_attenuation')
        self.heading_error_prev = 0.0
        self.heading_error_sum = 0.0

        self.controller_initialized = False

        self.check_obstacles = CheckObstacles()

    def updateLeaderPose(self, pose):
        """
            Update Leader Pose
        """
        self.thread_lock.acquire()
        x = pose.pose.pose.position.x
        y = pose.pose.pose.position.y
        q = pose.pose.pose.orientation
        roll, pitch, yaw = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        if len(self.path) == 0:
            # Path starting point
            distance = 0
            self.path.append([x, y, yaw, distance])
        else:
            x_last, y_last, theta, distance_last = self.path[-1]
            distance = distance_last + np.sqrt( np.power(x - x_last, 2) + np.power(y - y_last, 2) )
            # Append to path if movment is observed. Else ignore
            if distance > 0.1: 
                self.path.append([x, y, yaw, distance]) 
            
        self.thread_lock.release()

    def updatePose(self, pose):
        """
            Update follower pose
        """
        q = pose.pose.pose.orientation
        roll, pitch, yaw = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.thread_lock.acquire()
        self.pose = [pose.pose.pose.position.x, pose.pose.pose.position.y, yaw]
        self.thread_lock.release()

    def readLeaderPath(self):
        self.thread_lock.acquire()
        path = copy.deepcopy(self.path)
        self.thread_lock.release()
        return path
    
    def readPose(self):
        self.thread_lock.acquire()
        pose = copy.deepcopy(self.pose)
        self.thread_lock.release()
        return pose

    def extract_poses(self):
        """
            Returns current pose of follower and target pose

            Returns:    current_pose, target_pose
                        (x, y, theta), (x, y, theta)

            Throws: ValueError exception 

        """
        if len(self.path) == 0:
            raise ValueError("Path Empty")
            return
    
        path = np.array(self.readLeaderPath())

        # Path distance at target position
        target_dist = path[-1,3] - self.distance
        # Filter by distance
        far_pose_ids = np.where(path[:,3] < target_dist) #path[path[:,3] < target_dist]

        if len(far_pose_ids[0]) != 0:
            target_pose_id = far_pose_ids[0][-1]
            target_pose = path[target_pose_id,:3]

            # Delete Path history
            self.thread_lock.acquire()
            self.path = path[target_pose_id:,:].tolist()
            self.thread_lock.release()
        else:
            # No target
            raise ValueError("No target")
            return

        current_pose = np.array(self.readPose())
        return current_pose, target_pose

    def extract_path_following_poses(self):
        """
            Returns current pose of follower and next target pose in order to follow leader path

            Returns:    current_pose, target_pose
                        (x, y, theta), (x, y, theta)

            Throws: ValueError exception
        """
        current_pose = np.array(self.readPose())
        x, y, theta = current_pose

        if len(self.path) == 0:
            raise ValueError("Path Empty")
            return

        # Initialize target pose to current pose
        target_pose = current_pose

        path = self.readLeaderPath()
        # Path distance at target position
        target_dist = path[-1][3] - self.distance

        while (len(self.path) > 0):
            x_n, y_n, theta_n, dist_n = np.array(path[0])

            dist_to_next_point = np.sqrt( np.power(x - x_n, 2) + np.power(y - y_n, 2) )
            if dist_to_next_point < self.distance_tolerance:
                del path[0]
            else:
                # Check distance from next point to leader
                if dist_n < target_dist:
                    # Next point is still behind leader gap. Update target_pose
                    target_pose = x_n, y_n, theta_n
                    self.target_retries = 0
                else:
                    # Next point too close to leader. Stay in current position
                    target_pose = current_pose
                break
        
        # Current pose probably too close to leader. Stay in current position

        # Update path
        self.thread_lock.acquire()
        self.path = path
        self.thread_lock.release()

        return current_pose, target_pose
    
   
    def controller(self):

        try:
            current_pose, target_pose = self.extract_poses()
        except Exception as e: 
            # print(e)
            return

        x, y, theta = current_pose
        x_t, y_t, theta_t = target_pose
        print (target_pose)

        target_heading = math.atan2(y_t-y, x_t-x)
        heading_error = self.normalize_angle(target_heading - theta)

        self.heading_error_sum = self.normalize_angle(self.heading_error_sum + heading_error * (rospy.get_time() - self.controller_last_timestamp))
        heading_error_difference = heading_error - self.heading_error_prev
        angular_feedback = self.angular_Kp * heading_error + self.angular_Kd * heading_error_difference + self.angular_Ki * self.heading_error_sum

        self.controller_last_timestamp = rospy.get_time()
        self.heading_error_prev = heading_error

        distance_error = np.sqrt( np.power(x_t-x, 2) + np.power(y_t-y, 2) )
        if distance_error < self.distance_tolerance:
            distance_error = 0.0
        
        #### Generating Velocity ####
        cmd_vel = Twist()
        
        # Angular Velocity
        cmd_vel.angular.z = angular_feedback
        cmd_vel.angular.z = np.clip(cmd_vel.angular.z, a_min=-self.max_angular_velocity, a_max=self.max_angular_velocity)
        # Linear Velocity
        cmd_vel.linear.x = distance_error * self.linear_velocity_gain
        cmd_vel.linear.x = np.clip(cmd_vel.linear.x, a_min=-self.max_linear_velocity, a_max=self.max_linear_velocity)
        # Linear Velocity attenuation
        linear_velocity_attenuation = np.clip( self.obs_vel_attenuation * math.fabs(cmd_vel.angular.z), a_min=1, a_max=10)
        cmd_vel.linear.x /= linear_velocity_attenuation

        # # Obstacle Avoidance
        # if self.obstacle_avoidance:
        #     obstacle_avoidance_steering, front_proximity = self.check_obstacles.caculate_steering()
        #     cmd_vel.angular.z += obstacle_avoidance_steering
        #     obstacle_avoidance_linear_velocity_attenuation = np.clip( self.obs_vel_attenuation * math.fabs(obstacle_avoidance_steering), a_min=1, a_max=50) / front_proximity
        #     cmd_vel.linear.x /= obstacle_avoidance_linear_velocity_attenuation

        # print ("LinErr: %.3f, LinVel: %.3f, AngErr: %.3f, AngVel: %.3f, ObsSteer: %0.3f" %(distance_error, cmd_vel.linear.x, heading_error, cmd_vel.angular.z, obstacle_avoidance_steering))
        
        # Limiting Velocities
        cmd_vel.linear.x = np.clip(cmd_vel.linear.x, a_min=-self.max_linear_velocity, a_max=self.max_linear_velocity)
        cmd_vel.angular.z = np.clip(cmd_vel.angular.z, a_min=-self.max_angular_velocity, a_max=self.max_angular_velocity)

        # print ("LinErr: %.3f, LinVel: %.3f, AngErr: %.3f, AngVel: %.3f, ObsSteer: %0.3f" %(distance_error, cmd_vel.linear.x, heading_error, cmd_vel.angular.z, obstacle_avoidance_steering))

        self.publish_velocity(cmd_vel)

    def strict_controller(self):
        if not self.controller_initialized:
            try:
                current_pose, target_pose = self.extract_poses()
                self.controller_initialized = True
            except Exception as e: 
                # print(e)
                return
        else:
            try:
                current_pose, target_pose = self.extract_path_following_poses()
            except Exception as e: 
                # print(e)
                return
            

        x, y, theta = current_pose
        x_t, y_t, theta_t = target_pose

        target_heading = math.atan2(y_t-y, x_t-x)
        heading_error = self.normalize_angle(target_heading - theta)

        angular_feedback = heading_error
        distance_error = np.sqrt( np.power(x_t-x, 2) + np.power(y_t-y, 2) )
        
        #### Generating Velocity ####
        cmd_vel = Twist()
        
        # Angular Velocity
        cmd_vel.angular.z = angular_feedback
        cmd_vel.angular.z = np.clip(cmd_vel.angular.z, a_min=-self.max_angular_velocity, a_max=self.max_angular_velocity)
        # Linear Velocity
        if distance_error > self.distance_tolerance:
            cmd_vel.linear.x = distance_error * self.linear_velocity_gain
        else:
            cmd_vel.linear.x = 0
        
        # Linear Velocity attenuation
        linear_velocity_attenuation = np.clip( self.linear_velocity_attenuation * math.fabs(cmd_vel.angular.z), a_min=1, a_max=10)
        cmd_vel.linear.x /= linear_velocity_attenuation

        # Obstacle Avoidance
        if self.obstacle_avoidance:
            state, cmd_vel.linear.x, cmd_vel.angular.z = self.check_obstacles.avoid_obstacles(cmd_vel.linear.x, cmd_vel.angular.z)
            if not state:
                print ('Target Skipped: ', target_pose)
                self.thread_lock.acquire()
                del self.path[0]
                self.thread_lock.release()
    
        # print ("LinErr: %.3f, LinVel: %.3f, AngErr: %.3f, AngVel: %.3f, ObsSteer: %0.3f" %(distance_error, cmd_vel.linear.x, heading_error, cmd_vel.angular.z, obstacle_avoidance_steering))
        
        # Limiting Velocities
        cmd_vel.linear.x = np.clip(cmd_vel.linear.x, a_min=-self.max_linear_velocity, a_max=self.max_linear_velocity)
        cmd_vel.angular.z = np.clip(cmd_vel.angular.z, a_min=-self.max_angular_velocity, a_max=self.max_angular_velocity)

        # print ("LinErr: %.3f, LinVel: %.3f, AngErr: %.3f, AngVel: %.3f, ObsSteer: %0.3f" %(distance_error, cmd_vel.linear.x, heading_error, cmd_vel.angular.z, obstacle_avoidance_steering))
        self.publish_velocity(cmd_vel)

    def circular_follower(self):
        if not self.controller_initialized:
            try:
                current_pose, target_pose = self.extract_poses()
                self.controller_initialized = True
            except Exception as e: 
                # print(e)
                return
        else:
            try:
                current_pose, target_pose = self.extract_path_following_poses()
            except Exception as e: 
                # print(e)
                return

        x, y, theta = current_pose
        x_t, y_t, theta_t = target_pose
        
        distance_error = np.sqrt( np.power(x_t-x, 2) + np.power(y_t-y, 2) )

        targetX = x_t - x
        targetY = y_t -y
        targetW = self.normalize_angle(theta_t - theta)
        A = targetW + math.pi/2
        B = math.atan2(targetY, targetX) + (math.pi/2)
        targetAngle = math.atan2(targetY, targetX)

        originX = ( (-0.5)*y_t*math.cos(A)*math.cos(B) + x_t*( math.sin(A)*math.cos(B) - (math.cos(A)*math.sin(B))/2 ) ) / math.sin(A-B)
        originY = ( y_t*( (math.sin(A)*math.cos(B))/2 - math.cos(A)*math.sin(B)) + (x_t/2)*math.sin(A)*math.sin(B) ) / math.sin(A-B)
        path_radius = np.sqrt( np.power(originX, 2) + np.power(originY, 2) )

        originAngle = math.atan2(originY, originX)
        tangentAngle = self.normalize_angle(originAngle + math.pi/2)


        # Self rotation direction calculation
        crossProdOriginSelf = (-originX) * math.sin(tangentAngle) - (-originY) * math.cos(tangentAngle)
        if crossProdOriginSelf > 0:
            selfRotationCCW = True
        else:
            selfRotationCCW = False

        # Target rotation direction calculation
        crossProdOriginTarget = (targetX - originX) * math.sin(targetW) - (targetY - originY) * math.cos(targetW)
        if crossProdOriginTarget > 0:
            targetRotationCCW = True
        else:
            targetRotationCCW = False

        if selfRotationCCW == targetRotationCCW:
            heading_error = tangentAngle
        else:
            heading_error = self.normalize_angle(tangentAngle + math.pi)

        angular_feedback = 0
        # Create velocity
        cmd_vel = Twist()
        if (math.fabs(heading_error) > math.pi/2):
            cmd_vel.angular.z = heading_error * 0.1
        else:
            cmd_vel.linear.x = distance_error 
            
            self.heading_error_sum = self.heading_error_sum + heading_error * (rospy.get_time() - self.controller_last_timestamp)
            if math.fabs(self.heading_error_sum) > math.pi: self.heading_error_sum = 0    # Integrator Anti-windup
            self.controller_last_timestamp = rospy.get_time()
            heading_error_difference = heading_error - self.heading_error_prev
            if math.fabs(heading_error_difference) > math.pi : heading_error_difference = 0     # Overflow control
            self.heading_error_prev = heading_error
            angular_feedback = self.angular_Kp * heading_error + self.angular_Kd * heading_error_difference + self.angular_Ki * self.heading_error_sum

            if targetRotationCCW:
                cmd_vel.angular.z = cmd_vel.linear.x / path_radius + angular_feedback
            else:
                cmd_vel.angular.z = -cmd_vel.linear.x / path_radius + angular_feedback

        print ("%.3f,  %.3f,  %.3f,  %.3f,  %.3f,  %.3f,  %.3f,  %.3f, %.3f" %(targetX, targetY, targetW, originX, originY, path_radius, cmd_vel.linear.x, cmd_vel.angular.z, angular_feedback))

        # # Obstacle Avoidance
        # if self.obstacle_avoidance:
        #     obstacle_avoidance_steering, front_proximity = self.check_obstacles.caculate_steering()
        #     cmd_vel.angular.z += obstacle_avoidance_steering
        #     # obstacle_avoidance_linear_velocity_attenuation = np.clip( self.obs_vel_attenuation * math.fabs(obstacle_avoidance_steering), a_min=1, a_max=50) / front_proximity
        #     cmd_vel.linear.x *= front_proximity

        #     if math.fabs(obstacle_avoidance_steering) > 1:
        #         print ('Target Skipped: ', target_pose)
        #         self.thread_lock.acquire()
        #         del self.path[0]
        #         self.thread_lock.release()

        #     # if front_proximity < 0.3:
        #     #     # High obstacle density
        #     #     self.target_retries += 1
        #     #     if self.target_retries > 20:
        #     #         # Try for 2 seconds and then skip target
        #     #         print ('Target Skipped: ', target_pose)
        #     #         self.thread_lock.acquire()
        #     #         del self.path[0]
        #     #         self.thread_lock.release()

        #     else:
        #         self.target_retries = 0

        # # Linear Velocity attenuation
        # linear_velocity_attenuation = np.clip( self.obs_vel_attenuation * math.fabs(cmd_vel.angular.z), a_min=1, a_max=10)
        # cmd_vel.linear.x /= linear_velocity_attenuation

        # print ("LinErr: %.3f, LinVel: %.3f, AngErr: %.3f, AngVel: %.3f, ObsSteer: %0.3f" %(distance_error, cmd_vel.linear.x, heading_error, cmd_vel.angular.z, obstacle_avoidance_steering))
        
        # # Limiting Velocities
        # cmd_vel.linear.x = np.clip(cmd_vel.linear.x, a_min=-self.max_linear_velocity, a_max=self.max_linear_velocity)
        # cmd_vel.angular.z = np.clip(cmd_vel.angular.z, a_min=-self.max_angular_velocity, a_max=self.max_angular_velocity)

        # print ("LinErr: %.3f, LinVel: %.3f, AngErr: %.3f, AngVel: %.3f, ObsSteer: %0.3f" %(distance_error, cmd_vel.linear.x, heading_error, cmd_vel.angular.z, obstacle_avoidance_steering))
        self.publish_velocity(cmd_vel)


    def normalize_angle(self, angle):
        """
            Returns an angle between -PI and PI
        """
        angle = math.fmod(angle, 2 * math.pi)

        if (angle > math.pi):
            angle = angle - 2 * math.pi
        elif (angle < -math.pi):
            angle = angle + 2 * math.pi
        return angle

    def stop(self):
        self.timer.stop()
        self.check_obstacles.terminate()
        

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

    
        
