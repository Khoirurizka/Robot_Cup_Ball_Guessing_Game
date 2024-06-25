#!/usr/bin/env python

import rospy
from kinova_msgs.msg import PoseVelocity
from geometry_msgs.msg import PoseStamped
import time

def move_robot():
    rospy.init_node('move_robot_node', anonymous=True)
    pub = rospy.Publisher('/j2n6s300_driver/in/cartesian_velocity', PoseVelocity, queue_size=10)

    rate = rospy.Rate(10)

    velocity_cmd = PoseVelocity()

    velocity_cmd.twist_linear_x = 0.1
    velocity_cmd.twist_linear_y = 0.0
    velocity_cmd.twist_linear_z = 0.0

    velocity_cmd.twist_angular_x = 0.0
    velocity_cmd.twist_angular_y = 0.0
    velocity_cmd.twist_angular_z = 0.0

    start_time = time.time()
    duration = 5

    while not rospy.is_shutdown():
        if time.time() - start_time > duration:
            break

        pub.publish(velocity_cmd)
        rate.sleep()

    velocity_cmd.twist_linear_x = 0.0
    velocity_cmd.twist_linear_y = 0.0
    velocity_cmd.twist_linear_z = 0.0
    pub.publish(velocity_cmd)

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
