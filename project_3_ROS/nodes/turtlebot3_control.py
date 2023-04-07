#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

"""
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
"""


if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot3_control')
        pub = rospy.Publisher(name='cmd_vel', data_class=Twist, queue_size=10)
        turtlebot3_model = rospy.get_param("model", "burger")

        twist = Twist()

        control_linear_vel = 1
        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        control_angular_vel = 0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

        pub.publish(twist)

        
    except rospy.ROSInterruptException:
        pass
