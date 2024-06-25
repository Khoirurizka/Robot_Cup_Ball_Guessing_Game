#!/usr/bin/env python3

import threading
import os
import rospy
from kinova_msgs.srv import AddPoseToCartesianTrajectory, AddPoseToCartesianTrajectoryRequest, ClearTrajectories, ClearTrajectoriesRequest

class task_robot(threading.Thread):
    def __init__(self,name):
        threading.Thread.__init__(self)
        rospy.init_node('move_robot_node', anonymous=True)
        self.name = name
                
        self.rate = rospy.Rate(10)
        
        self.home_pos = [0.0, -0.25, 0.4, 0.0,0.0,0.0]

    def homing(self):
        # Initialize the ROS node


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
            self.request = AddPoseToCartesianTrajectoryRequest()
            self.request.X = self.home_pos[0]
            self.request.Y = self.home_pos[1]
            self.request.Z = self.home_pos[2]
            self.request.ThetaX = self.home_pos[3]
            self.request.ThetaY = self.home_pos[4]
            self.request.ThetaZ = self.home_pos[5]

            # Call the service
            self.response_clear = clear_pose(request_clear)
            self.response = add_pose_service(self.request)
            rospy.loginfo("Service call successful: %s", self.response)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
    #
    def run(self):
        #self.homing()
        print("Task 1 assigned to thread: {}".format(threading.current_thread().name))
        rospy.loginfo("ROS Node is running.")
        while not rospy.is_shutdown():
            
            self.homing()
            
            self.rate.sleep()


