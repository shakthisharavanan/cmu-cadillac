#!/usr/bin/env python  
import roslib
import rospy
import tf
from nav_msgs.msg import Odometry

class odom_tf(object):
    def __init__(self):
        rospy.init_node('my_tf_broadcaster')
        self.x = 0
        self.y = 0 
        self.z = 0
        self.q0 = 0
        self.q1 = 0
        self.q2 = 0
        self.q3 = 1
        rospy.Subscriber('odom', Odometry, self.odom_cb, queue_size = 10)
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10.0)

    def odom_cb(self,odom):
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        self.z = odom.pose.pose.position.z
        self.q0 = odom.pose.pose.orientation.x
        self.q1 = odom.pose.pose.orientation.y
        self.q2 = odom.pose.pose.orientation.z
        self.q3 = odom.pose.pose.orientation.w 

    def spin(self):
        while not rospy.is_shutdown():
            self.br.sendTransform((self.x, self.y, self.z),
                             (self.q0, self.q1, self.q2, self.q3),
                             rospy.Time.now(),
                             "base_link", "odom",)
            self.rate.sleep()

if __name__ == '__main__':
    ctrl = odom_tf()
    try:
        ctrl.spin()
    except rospy.ROSInterruptException:
        pass