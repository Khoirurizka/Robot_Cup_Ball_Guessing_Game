#!/usr/bin/env python3

import rospy
from kinova_msgs.srv import AddPoseToCartesianTrajectory, AddPoseToCartesianTrajectoryRequest, ClearTrajectories, ClearTrajectoriesRequest


def kinova_move(k_x,k_y,k_z,r_deg_x,r_deg_y,r_deg_z):
    # Initialize the ROS node
    rospy.init_node('kinova_move_node')

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
        request.X = k_x
        request.Y = -k_y
        request.Z = k_z
        request.ThetaX = r_deg_x*3.14/180
        request.ThetaY = r_deg_y*3.14/180
        request.ThetaZ = r_deg_z*3.14/180

        # Call the service
        response_clear = clear_pose(request_clear)
        response = add_pose_service(request)
        rospy.loginfo("Service call successful: %s", response)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        
def kinova_homing_c():
    kinova_move(0.0,0.5,0.4,180,0,-90)
def kinova_pick_cup_1():
    kinova_move(0.0,0.5,0.4,180,0,-90)
def kinova_pick_cup_2():
    kinova_move(0.0,0.5,0.4,180,0,-90)
def kinova_pick_cup_3():
    kinova_move(0.0,0.5,0.4,180,0,-90)

def kinova_place_cup_1():
    kinova_move(0.0,0.5,0.4,180,0,-90)
def kinova_place_cup_2():
    kinova_move(0.0,0.5,0.4,180,0,-90)
def kinova_place_cup_3():
    kinova_move(0.0,0.5,0.4,180,0,-90)

if __name__ == '__main__':
    try:
        kinova_pick_cup_1()
    except rospy.ROSInterruptException:
        pass

