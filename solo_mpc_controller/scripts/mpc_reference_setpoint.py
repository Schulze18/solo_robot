#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler

class MpcReference:         	          

    def __init__(self):
        self.time_rate = 0.01
        self.rate = rospy.Rate(1/self.time_rate)
        self.ref_pub = rospy.Publisher('/com_pose_ref', PoseStamped, queue_size=10)
        self.ref_z = [0.3, 0.2, 0.25, 0.30, 0.30]
        self.ref_phi = [0.0, 0.0, 0.0, 0.1, 0.1]
        self.ref_theta = [0.0, 0.0, 0.1, 0.0, -0.1]
        self.time_tick = 10/self.time_rate
        self.time_count = 0
        self.ref_point = Point()
        self.index = 0
        self.msg_pose = PoseStamped()

    def pubReference(self):
        self.msg_pose.header.stamp = rospy.Time.now()
        self.msg_pose.header.frame_id = "odom"
        self.msg_pose.pose.position = self.ref_point
        self.msg_pose.pose.orientation = Quaternion(self.ref_quat[0], self.ref_quat[1], self.ref_quat[2], self.ref_quat[3])
        self.ref_pub.publish(self.msg_pose)

    def run(self):

        while not rospy.is_shutdown():
            self.time_count = self.time_count + 1
            if (self.time_count > 4*self.time_tick):
                self.index = 4
                if (self.time_count == self.index*self.time_tick + 1):
                    rospy.loginfo("Phase #%d | z: %.2f; phi: %.2f theta: %.2f", self.index, self.ref_z[self.index], self.ref_phi[self.index], self.ref_theta[self.index])
            elif (self.time_count > 3*self.time_tick):
                self.index = 3
                if (self.time_count == self.index*self.time_tick + 1):
                    rospy.loginfo("Phase #%d | z: %.2f; phi: %.2f theta: %.2f", self.index, self.ref_z[self.index], self.ref_phi[self.index], self.ref_theta[self.index])
            
            elif (self.time_count > 2*self.time_tick):
                self.index = 2
                if (self.time_count == self.index*self.time_tick + 1):
                    rospy.loginfo("Phase #%d | z: %.2f; phi: %.2f theta: %.2f", self.index, self.ref_z[self.index], self.ref_phi[self.index], self.ref_theta[self.index])

            elif (self.time_count > self.time_tick):
                self.index = 1
                if (self.time_count == self.index*self.time_tick + 1):
                    rospy.loginfo("Phase #%d | z: %.2f; phi: %.2f theta: %.2f", self.index, self.ref_z[self.index], self.ref_phi[self.index], self.ref_theta[self.index])
            else:
                self.index = 0
                if (self.time_count == self.index*self.time_tick + 1):
                    rospy.loginfo("Phase #%d | z: %.2f; phi: %.2f theta: %.2f", self.index, self.ref_z[self.index], self.ref_phi[self.index], self.ref_theta[self.index])

            self.ref_point.z = self.ref_z[self.index]
            self.ref_quat = quaternion_from_euler(self.ref_phi[self.index], self.ref_theta[self.index], 0)
            
            self.pubReference()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node("mpc_reference")	      
        mpc = MpcReference()
        mpc.run()
    except rospy.ROSInterruptException:
        pass
