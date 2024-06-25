#!/usr/bin/env python3

import rospy
from kinova_msgs.srv import AddPoseToCartesianTrajectory, AddPoseToCartesianTrajectoryRequest, ClearTrajectories, ClearTrajectoriesRequest

home_pos = [0.0, -0.25, 0.4, 0.0,0.0]


def move_robot():
    # Initialize the ROS node
    rospy.init_node('move_robot_node')

    # Wait for the service to be available
    rospy.wait_for_service('/j2n6s300_driver/in/clear_trajectories')
    rospy.wait_for_service('/j2n6s300_driver/in/add_pose_to_Cartesian_trajectory')

    try:
        # Create a service proxy
        clear_pose = rospy.ServiceProxy('/j2n6s300_driver/in/clear_trajectories', ClearTrajectories)
        add_pose_service = rospy.ServiceProxy('/j2n6s300_driver/in/add_pose_to_Cartesian_trajectory', AddPoseToCartesianTrajectory)
        
        # Clear traj
        request_clear = ClearTrajectoriesRequest()
        # Create a request object
        request = AddPoseToCartesianTrajectoryRequest()
        request.X = 0.0
        request.Y = -0.25
        request.Z = 0.4
        request.ThetaX = 3.1
        request.ThetaY = 0.0
        request.ThetaZ = 0.0

        # Call the service
        response_clear = clear_pose(request_clear)
        response = add_pose_service(request)
        rospy.loginfo("Service call successful: %s", response)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass

