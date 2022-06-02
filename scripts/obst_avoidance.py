#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import time
import rospy
from visualization_msgs.msg import Marker
import numpy as np

import tf

from nav_msgs.msg import Odometry

from perception.srv import emergency, emergencyResponse


def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

def move_to_point(current, goal, Kv=1.0, Kw = 0.6):

    # A proportional controller which sets a velocity command to move from current position to goal (u = Ke)
    # Make it sequential to avoid strange curves. First correct orientation and then distance. 
    # Hint: use wrap_angle function to maintain yaw in [-pi, pi]

    dx = goal[0]- current[0]
    dy = goal[1] - current[1]
    d_theta = wrap_angle(np.arctan2(dy, dx) - current[2])

    v = Kv*np.hypot(dx, dy) if np.abs(d_theta) < 0.1 else 0     # linear velocity
    w = Kw*d_theta                                             # angular velocity

    # v = Kv*np.hypot(dx, dy)                                  # linear velocity
    # w = Kw*d_theta       

    v = v if v < 0.08 else 0.08                                  # limiting linear velocity

    return v, w

class obstacle_avoidance():

    def __init__(self, odom_topic, emergency_srv_name, safety_cmd_vel_topic):

        # Safety velocity command oublisher
        self.vel_pub = rospy.Publisher(safety_cmd_vel_topic, Twist, queue_size=1)

        # Subscriber to the robot's odometry topic
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.get_odom) # : subscriber to odom_topic  

        # Emergency server to handle the emergency signal comming from the risk assessment node
        self.emergency_server = rospy.Service(emergency_srv_name, emergency, self.safety_controller)

        self.safety_marker_pub = rospy.Publisher("/safety_pose", Marker, queue_size = 2)

        self.control_lock_flag = True

        self.current_pose = None

    def get_odom(self, odom):

        # Subroutine to retrieve odometry information and know the current state of the mobile base

        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])

    def safety_controller(self, req):

        if self.control_lock_flag:

            self.control_lock_flag = False

            signal, min_risk_obs = int(req.signal), np.asarray([req.x, req.y, req.z])


            if signal == 0:

                # Safety distance to move forward (25% of the distance from the robot to the obstacle with lowest risk)

                safety_distance = np.linalg.norm(self.current_pose[0:2] - min_risk_obs[0:2])/4
                
                safety_theta = np.arctan2(min_risk_obs[1] - self.current_pose[1], min_risk_obs[0] - self.current_pose[0]) 

                safe_goal = np.array([self.current_pose[0] + safety_distance*np.cos(safety_theta), self.current_pose[1] + safety_distance*np.sin(safety_theta)])

                self.safety_position_vis(safe_goal[0], safe_goal[1], 0)
                
                safety_vel_msg = Twist()

                while(np.linalg.norm(self.current_pose[0:2] - safe_goal) > 0.2):

                    safety_vel_msg.linear.x , safety_vel_msg.angular.z  = move_to_point(self.current_pose, safe_goal, 1.2, 0.3)

                    self.vel_pub.publish(safety_vel_msg)


            elif signal == 1:

                # Safety distance to move backwards (25%)

                safety_distance = np.linalg.norm(self.current_pose[0:2] - min_risk_obs[0:2])/4
                
                safety_theta = np.arctan2(min_risk_obs[1] - self.current_pose[1], min_risk_obs[0] - self.current_pose[0]) 

                safe_goal = np.array([self.current_pose[0] - safety_distance*np.cos(safety_theta), self.current_pose[1] - safety_distance*np.sin(safety_theta)])

                self.safety_position_vis(safe_goal[0], safe_goal[1], 0)
                
                safety_vel_msg = Twist()

                while(np.linalg.norm(self.current_pose[0:2] - safe_goal) > 0.2):

                    safety_vel_msg.linear.x , safety_vel_msg.angular.z  = move_to_point(self.current_pose, safe_goal, 1.2, 0.3)

                    self.vel_pub.publish(safety_vel_msg)

        self.control_lock_flag = True # Unlock the service to be used later on

        return emergencyResponse(True)


    def safety_position_vis(self, x, y, z):

        marker = Marker()

        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 3
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.3

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        self.safety_marker_pub.publish(marker)

            


if __name__ == "__main__":

    print("OBSTACLE AVOIDANCE NODE IGNITED")

    rospy.init_node('obstacle_avoidance_node', anonymous=True)

    # Required Topics
    odom_topic = rospy.get_param("/tbot_odometry_topic")
    emergency_srv_name = rospy.get_param("/emergency_service")
    safety_cmd_vel_topic = rospy.get_param("/safety_vel_cmd_topic")

    # node initialization
    node = obstacle_avoidance(odom_topic, emergency_srv_name, safety_cmd_vel_topic)
    
    rospy.spin()