#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point, TwistStamped, Vector3
from std_msgs.msg import Time
import tf
import math
import numpy as np

class vrpnVelCalc:

    def __init__(self):
        # Define subscribers
        self.pose_sub = rospy.Subscriber('/vrpn_client_node/oflow_xaru8145/pose', PoseStamped, self.pose_cb, queue_size=10)
        # Define publishers
        self.vel_pub = rospy.Publisher('/vrpn_velocity/optitrack_frame/raw', TwistStamped, queue_size=10)
        self.filt_vel_pub = rospy.Publisher('/vrpn_velocity/optitrack_frame/filtered', TwistStamped, queue_size=10)
        self.body_vel_pub = rospy.Publisher('/vrpn_velocity/oflow_xaru8145_frame/filtered', TwistStamped, queue_size=10)

        # Create messages
        self.pose_msg = PoseStamped()
        self.vel_msg = TwistStamped()
        self.last_pose_msg = PoseStamped()
        self.last_vel_msg = TwistStamped()
        self.filt_vel_msg = TwistStamped()
        self.last_filt_vel_msg = TwistStamped()
        self.body_vel_msg = TwistStamped()

        # Set the LPF constant for filtering the velocity
        self.alpha = .9

        # Initialize all the vectors
        self.last_pose_msg.header.stamp = rospy.Time.now()
        self.last_pose_msg.pose.position = Point(0.0, 0.0, 0.0)
        self.last_yaw = 0
        self.last_vel_msg.header.stamp= rospy.Time.now()
        self.vel_msg.twist.linear= Vector3(0.0, 0.0, 0.0)
        self.vel_msg.twist.angular= Vector3(0.0, 0.0, 0.0)
        self.filt_vel_msg.header.stamp= rospy.Time.now()
        self.filt_vel_msg.twist.linear= Vector3(0.0, 0.0, 0.0)
        self.filt_vel_msg.twist.angular= Vector3(0.0, 0.0, 0.0)

        while not rospy.is_shutdown():
            rospy.spin()

    def pose_cb(self, msg):
        current_pose_array = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        last_pose_array = np.array([self.last_pose_msg.pose.position.x, self.last_pose_msg.pose.position.y, self.last_pose_msg.pose.position.z])
        current_quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)

        # Compute the time difference
        time_diff = msg.header.stamp - self.last_pose_msg.header.stamp
        time_diff_s = rospy.Time(time_diff.secs, time_diff.nsecs).to_sec()
#        print(time_diff_s)
        if (time_diff_s > .005):
            # Find the current yaw angle
            current_quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
            euler_angle = tf.transformations.euler_from_quaternion(current_quat)
            current_yaw = euler_angle[2]

            # Compute the pose difference
            pose_diff_m = current_pose_array - last_pose_array
            # Compute angle difference and wrap results
            if (current_yaw-self.last_yaw>math.pi):
                yaw_diff_m = (current_yaw-2*math.pi)-self.last_yaw
            elif (current_yaw-self.last_yaw<-math.pi):
                yaw_diff_m = (current_yaw+2*math.pi)-self.last_yaw
            else:
                yaw_diff_m = current_yaw - self.last_yaw

            # Numerical differentiation
            diff_vel_mps = np.divide(pose_diff_m, time_diff_s)
            diff_rate_mps = np.divide(yaw_diff_m, time_diff_s)

            # Publish the velocity message
            self.vel_msg.twist.linear = Vector3(diff_vel_mps[0], diff_vel_mps[1], diff_vel_mps[2])
            self.vel_msg.twist.angular.z = diff_rate_mps
            self.vel_msg.header.stamp = rospy.Time.now()
            self.vel_pub.publish(self.vel_msg)

            # Filter the velocity output
            self.filt_vel_msg.header.stamp = rospy.Time.now()
            self.filt_vel_msg.twist.linear.x = (1-self.alpha)*self.vel_msg.twist.linear.x + self.alpha*self.last_filt_vel_msg.twist.linear.x
            self.filt_vel_msg.twist.linear.y = (1-self.alpha)*self.vel_msg.twist.linear.y + self.alpha*self.last_filt_vel_msg.twist.linear.y
            self.filt_vel_msg.twist.linear.z = (1-self.alpha)*self.vel_msg.twist.linear.z + self.alpha*self.last_filt_vel_msg.twist.linear.z
            self.filt_vel_msg.twist.angular.z = (1-self.alpha)*self.vel_msg.twist.angular.z + self.alpha*self.last_filt_vel_msg.twist.angular.z
            self.filt_vel_pub.publish(self.filt_vel_msg)

            # Rotate the velocity into the vehicle 1 frame (yaw rotation)
            self.body_vel_msg.header.stamp = rospy.Time.now()
            self.body_vel_msg.twist.linear.x = math.cos(current_yaw)*self.filt_vel_msg.twist.linear.x + math.sin(current_yaw)*self.filt_vel_msg.twist.linear.y
            self.body_vel_msg.twist.linear.y = -math.sin(current_yaw)*self.filt_vel_msg.twist.linear.x + math.cos(current_yaw)*self.filt_vel_msg.twist.linear.y
            self.body_vel_msg.twist.linear.z = self.filt_vel_msg.twist.linear.z
            self.body_vel_msg.twist.angular.z = self.filt_vel_msg.twist.angular.z
            self.body_vel_pub.publish(self.body_vel_msg)

            # Update last pose variable
            self.last_pose_msg = msg
            self.last_filt_vel_msg = self.filt_vel_msg
            self.last_yaw = current_yaw

if __name__ == '__main__':
    rospy.init_node('vrpn_velocity_node')
    try:
        velocity_compute = vrpnVelCalc()
    except:
        rospy.ROSInterruptException
    pass
