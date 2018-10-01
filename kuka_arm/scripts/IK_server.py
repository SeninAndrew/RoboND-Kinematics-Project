#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        r, p, y = symbols('r, p, y')

        DH = {
          alpha0: 0,      a0: 0,       q1: q1,         d1: 0.75,
          alpha1: -pi/2,  a1: 0.35,    q2: q2 - pi/2,  d2: 0,
          alpha2: 0,      a2: 1.25,    q3: q3,         d3: 0,
          alpha3: -pi/2,  a3: -0.054,  q4: q4,         d4: 1.5,
          alpha4: pi/2,   a4: 0,       q5: q5,         d5: 0,
          alpha5: -pi/2,  a5: 0,       q6: q6,         d6: 0,
          alpha6: 0,      a6: 0,       q7: 0,          d7: 0.303,
        }

        def GetRotationMatrixByDHParams(alpha, a, q, d):
            return Matrix([[cos(q),             -sin(q),               0.0,         a], 
                           [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                           [sin(q) * sin(alpha), cos(q) * sin(alpha),  cos(alpha),  cos(alpha) * d],
                           [0.0,                 0.0,                  0.0,         1.0]]) 

        def GetRotationMatrixByEuler(rx, ry, rz):
            mx = Matrix([[1.0,  0.0,      0.0], 
                         [0.0,  cos(rx), -sin(rx)],
                         [0.0,  sin(rx),  cos(rx)]])

            my = Matrix([[cos(ry),  0.0,      sin(ry)], 
                         [0.0,      1.0,      0.0],
                         [-sin(ry), 0.0 ,     cos(ry)]])

            mz = Matrix([[cos(rz), -sin(rz),  0.0], 
                         [sin(rz),  cos(rz),  0.0],
                         [0.0,      0.0,      1.0]])

            return mz * my * mx

        # Homogeneous matrix from neighboring joints 
        T0_1 = GetRotationMatrixByDHParams(alpha0, a0, q1, d1).subs(DH)
        T1_2 = GetRotationMatrixByDHParams(alpha1, a1, q2, d2).subs(DH)
        T2_3 = GetRotationMatrixByDHParams(alpha2, a2, q3, d3).subs(DH)
        T3_4 = GetRotationMatrixByDHParams(alpha3, a3, q4, d4).subs(DH)
        T4_5 = GetRotationMatrixByDHParams(alpha4, a4, q5, d5).subs(DH)
        T5_6 = GetRotationMatrixByDHParams(alpha5, a5, q6, d6).subs(DH)
        T6_7 = GetRotationMatrixByDHParams(alpha6, a6, q7, d7).subs(DH)

        # Correction from DH to Kuka 
        EE_corr = GetRotationMatrixByEuler(0,0,pi) * GetRotationMatrixByEuler(0,-pi/2,0)

        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7

        EE_orientation = GetRotationMatrixByEuler(r, p, y)

        EE_orientation_corrected = simplify(EE_orientation * EE_corr)

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

    	    # Extract end-effector position and orientation from request
    	    # px,py,pz = end-effector position
    	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            # Locate the wrist center            
            EE_pos = Matrix([[px], [py], [pz]])
            EE_orientation_corrected_subs = EE_orientation_corrected.subs({'r': roll, 'p': pitch, 'y': yaw})
            WC_pos = EE_pos - DH[d7] * EE_orientation_corrected_subs[:, 2]

            # Solving for the first 3 thetas
            theta1 = atan2(WC_pos[1], WC_pos[0])
            s1 = DH[d4]
            s2 = sqrt(pow(sqrt(WC_pos[0] * WC_pos[0] + WC_pos[1] * WC_pos[1]) - DH[a1], 2) + pow((WC_pos[2] - DH[d1]), 2)) 
            s3 = DH[a2]

            angle1 = acos((s2 * s2 + s3 * s3 - s1 * s1) / (2 * s2 * s3))    
            angle2 = acos((s1 * s1 + s3 * s3 - s2 * s2) / (2 * s1 * s3))
            angle3 = acos((s1 * s1 + s2 * s2 - s3 * s3 ) / (2 * s1 * s2))

            theta2 = pi/2. - angle1 - atan2(WC_pos[2] - DH[d1], sqrt(WC_pos[0] * WC_pos[0] + WC_pos[1] * WC_pos[1]) - DH[a1])
            theta3 = pi/2. - (angle2 + atan2(abs(DH[a3]), abs(DH[d4])))

            # Finding rotation matrix for the last 3 joints
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3: theta3})
            R3_6 = R0_3.transpose() * EE_orientation_corrected_subs

            # Euler angles from rotation matrix
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0]) 
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
