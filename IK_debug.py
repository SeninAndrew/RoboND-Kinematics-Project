from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!

    # Extract position and orientation of the EE
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x,
         req.poses[x].orientation.y,
         req.poses[x].orientation.z,
         req.poses[x].orientation.w])

    # Define the DH table
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')

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

    EE_pos = Matrix([[px],
                     [py],
                     [pz]])
    EE_orientation = GetRotationMatrixByEuler(roll, pitch, yaw)

    EE_orientation_corrected = EE_orientation * EE_corr

    # Locate the wrist center
    WC_pos = EE_pos - DH[d7] * EE_orientation_corrected [:, 2]

    # Solving for the first 3 thetas
    theta1 = atan2(WC_pos[1], WC_pos[0])
    s1 = DH[d4]
    s2 = sqrt(pow(sqrt(WC_pos[0] * WC_pos[0] + WC_pos[1] * WC_pos[1]) - DH[a1], 2) + pow((WC_pos[2] - DH[d1]), 2)) 
    s3 = DH[a2]

    angle1 = acos((s2 * s2 + s3 * s3 - s1 * s1) / (2 * s2 * s3))
    angle2 = acos((s1 * s1 + s3 * s3 - s2 * s2) / (2 * s1 * s3))
    angle3 = acos((s1 * s1 + s2 * s2 - s3 * s3 ) / (2 * s1 * s2))

    theta2 = pi/2. - angle1 - atan2(WC_pos[2] - DH[d1], sqrt(WC_pos[0] * WC_pos[0] + WC_pos[1] * WC_pos[1]) - DH[a1])
    theta3 = pi/2. - (angle2 + 0.036) # 0.036 accounts for sag in link4 of -0.054m

    # Finding rotation matrix for the last 3 joints
    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3: theta3})
    R3_6 = R0_3.transpose() * EE_orientation_corrected

    # Euler angles from rotation matrix
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0]) 

    # Calculating last 3 thetas from the rotation matrix

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    FK = T0_EE.evalf(subs={q1: theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6})


    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [WC_pos[0], WC_pos[1], WC_pos[2]] # <--- Load your calculated WC values in this array
    your_ee = [FK[0,3], FK[1,3], FK[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f (%04.8f/%04.8f)" % (t_1_e, theta1, test_case[2][0]))
    print ("Theta 2 error is: %04.8f (%04.8f/%04.8f)" % (t_2_e, theta2, test_case[2][1]))
    print ("Theta 3 error is: %04.8f (%04.8f/%04.8f)" % (t_3_e, theta3, test_case[2][2]))
    print ("Theta 4 error is: %04.8f (%04.8f/%04.8f)" % (t_4_e, theta4, test_case[2][3]))
    print ("Theta 5 error is: %04.8f (%04.8f/%04.8f)" % (t_5_e, theta5, test_case[2][4]))
    print ("Theta 6 error is: %04.8f (%04.8f/%04.8f)" % (t_6_e, theta6, test_case[2][5]))
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 2

    test_code(test_cases[test_case_number])
