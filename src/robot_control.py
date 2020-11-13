#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class robot_control:

    def __init__(self):
        rospy.init_node('robot_control', anonymous=True)
        rate = rospy.Rate(30) # 30hz
        self.joints = []
        # initialize publishers to send joint angles to the robot
        self.joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        # initialize subscribers to get joint angles from robot
        print("initializing joint_states_sub")
        self.joint_states_sub = rospy.Subscriber("/robot/joint_states", JointState,callback=self.joint_callback)
        t0 = rospy.get_time()

    def joint_callback(self,data):
        self.joints = np.array(data.position) # update joint angles from ros subscriber

    def getEndFKPos(self):
        # joints[0] -> rotate around z axis, 2.5m long
        # joints[1] -> rotate around x axis, 0m long
        # joints[2] -> rotate around y axis, 3.5m long ^ (1,2 are same positioned blue joint)
        # joints[3] -> rotate around x axis, 3m long
        q = self.joints

        
        ''' 
        # 4 rotation matrices

        R0 = np.array([ [np.cos(q[0]),-np.sin(q[0]), 0],
                        [np.sin(q[0]), np.cos(q[0]), 0],
                        [0, 0, 1]])
        R1 = np.array([ [1, 0, 0],
                        [0, np.cos(q[1]),-np.sin(q[1])],
                        [0, np.sin(q[1]), np.cos(q[1])]])
        R2 = np.array([ [np.cos(q[2]), 0, np.sin(q[2])],
                        [0, 1, 0],
                        [-np.sin(q[2]),0, np.cos(q[2])]])
        R3 = np.array([ [1, 0, 0],
                        [0, np.cos(q[3]),-np.sin(q[3])],
                        [0, np.sin(q[3]), np.cos(q[3])]])

        # transformations:
        # T01: origin -> link1, rotation by R0, distance of 0
        # T12: link1 -> link2, rotaion by R1, distance of 2.5m
        # T23: link2 -> link3, rotation by R2, distance of 0m
        # T34: link3 -> link4, rotation by R3, distance of 3.5m
        # T45: link4 -> end effector, no rotation, distance of 3m
        
        T01 = np.array([[R0[0,0],R0[0,1],R0[0,2], 0],
                        [R0[1,0],R0[1,1],R0[1,2], 0],
                        [R0[2,0],R0[2,1],R0[2,2], 0],
                        [0, 0, 0, 1]])
        T12 = np.array([[R1[0,0],R1[0,1],R1[0,2], 0],
                        [R1[1,0],R1[1,1],R1[1,2], 0],
                        [R1[2,0],R1[2,1],R1[2,2], 2.5],
                        [0, 0, 0, 1]])
        T23 = np.array([[R2[0,0],R2[0,1],R2[0,2], 0],
                        [R2[1,0],R2[1,1],R2[1,2], 0],
                        [R2[2,0],R2[2,1],R2[2,2], 0],
                        [0, 0, 0, 1]])
        T34 = np.array([[R3[0,0],R3[0,1],R3[0,2], 0],
                        [R3[1,0],R3[1,1],R3[1,2], 0],
                        [R3[2,0],R3[2,1],R3[2,2], 3.5],
                        [0, 0, 0, 1]])
        T45 = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 3],
                        [0, 0, 0, 1]])
        
        T = T01 @ T12 @ T23 @ T34 @ T45 # get our complete transformation
         # return the position given in T
        return [T[0,3],T[1,3],T[2,3]] # + [initial coord of link1/origin]?
        '''

        # this does the same as what is commented out above
        x = 3.5*(np.cos(q[0])*np.sin(q[2]) + np.sin(q[0])*np.sin(q[1]*np.cos(q[2]))) + 3*(np.sin(q[0])*np.cos(q[1])*np.sin(q[3]) + np.cos(q[3])*(np.cos(q[0])*np.sin(q[2]) + np.sin(q[0])*np.sin(q[1])*np.cos(q[2])))
        y = 3.5*(np.sin(q[0])*np.sin(q[2]) - np.cos(q[0])*np.sin(q[1])*np.cos(q[2])) + 3*(np.cos(q[3])*( np.sin(q[0])*np.sin(q[2]) - np.cos(q[0])*np.sin(q[1])*np.cos(q[2]) ) - np.cos(q[0])*np.cos(q[1])*np.sin(q[3]))
        z = 3.5*np.cos(q[1])*np.cos(q[2]) + 3*(np.cos(q[1])*np.cos(q[2])*np.cos(q[3]) - np.sin(q[1])*np.sin(q[3])) + 2.5

        return np.array([x,y,z])

    def getJacobian(self):


    def run(self):
        while not rospy.is_shutdown():
            if self.joints != []:
                print(self.getEndFKPos())



# run the code if the node is called
if __name__ == '__main__':
  try:
    rc = robot_control()
    rc.run()
  except rospy.ROSInterruptException:
    pass


