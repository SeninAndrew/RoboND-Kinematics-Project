# Robotic arm - Pick & Place project

## Denavit-Hartenberg Parameters

This project is using the Kuka KR 210 robotic arm which can be represented with the following list of joints (image credits: Udacity):

Based on UDRF specification:
![UDRF specification](https://raw.githubusercontent.com/SeninAndrew/RoboND-Kinematics-Project/master/imgs/dh_params.jpeg)

Based on DH specification:
![Denavit-Hartenberg Diagram](https://raw.githubusercontent.com/SeninAndrew/RoboND-Kinematics-Project/master/imgs/urdf_params.png)

Based on comparison of the specifications we convert the robot dimension information to the DH table as follows:

```
DH = {
  alpha0: 0,      a0: 0,       theta1: theta1,         d1: 0.75,
  alpha1: -pi/2,  a1: 0.35,    theta2: theta2 - pi/2,  d2: 0,
  alpha2: 0,      a2: 1.25,    theta3: theta3,         d3: 0,
  alpha3: -pi/2,  a3: -0.054,  theta4: theta4,         d4: 1.5,
  alpha4: pi/2,   a4: 0,       theta5: theta5,         d5: 0,
  alpha5: -pi/2,  a5: 0,       theta6: theta6,         d6: 0,
  alpha6: 0,      a6: 0,       theta7: 0,              d7: 0.303,
}
```

Since all the joints are revolute it is only the theta parameters which can vary.

## Joint transformation matrices

The transformation matrices can be calculated as follows:

```python
def GetRotationMatrixByDHParams(alpha, a, q, d):
    return Matrix([[cos(q),             -sin(q),               0.0,         a], 
                   [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                   [sin(q) * sin(alpha), cos(q) * sin(alpha),  cos(alpha),  cos(alpha) * d],
                   [0.0,                 0.0,                  0.0,         1.0]]) 
                   
T0_1 = GetRotationMatrixByDHParams(alpha0, a0, q1, d1).subs(DH)
T1_2 = GetRotationMatrixByDHParams(alpha1, a1, q2, d2).subs(DH)
T2_3 = GetRotationMatrixByDHParams(alpha2, a2, q3, d3).subs(DH)
T3_4 = GetRotationMatrixByDHParams(alpha3, a3, q4, d4).subs(DH)
T4_5 = GetRotationMatrixByDHParams(alpha4, a4, q5, d5).subs(DH)
T5_6 = GetRotationMatrixByDHParams(alpha5, a5, q6, d6).subs(DH)
T6_7 = GetRotationMatrixByDHParams(alpha6, a6, q7, d7).subs(DH)
```

The resulting transofrmations for each joint would be:
T0_1:
```
Matrix([
[cos(q1), -sin(q1), 0.0,    0],
[sin(q1),  cos(q1),   0,    0],
[      0,        0,   1, 0.75],
[    0.0,      0.0, 0.0,  1.0]])
```

T1_2:
```
Matrix([
[sin(q2),  cos(q2), 0.0, 0.35],
[      0,        0,   1,    0],
[cos(q2), -sin(q2),   0,    0],
[    0.0,      0.0, 0.0,  1.0]])
```

T2_3:
```
Matrix([
[cos(q3), -sin(q3), 0.0, 1.25],
[sin(q3),  cos(q3),   0,    0],
[      0,        0,   1,    0],
[    0.0,      0.0, 0.0,  1.0]])
```

T3_4:
```
Matrix([
[ cos(q4), -sin(q4), 0.0, -0.054],
[       0,        0,   1,    1.5],
[-sin(q4), -cos(q4),   0,      0],
[     0.0,      0.0, 0.0,    1.0]])
```

T4_5:
```
Matrix([
[cos(q5), -sin(q5), 0.0,   0],
[      0,        0,  -1,   0],
[sin(q5),  cos(q5),   0,   0],
[    0.0,      0.0, 0.0, 1.0]])
```

T5_6:
```
Matrix([
[ cos(q6), -sin(q6), 0.0,   0],
[       0,        0,   1,   0],
[-sin(q6), -cos(q6),   0,   0],
[     0.0,      0.0, 0.0, 1.0]])
```

T6_7 (End Effector):
```
Matrix([
[  1,   0, 0.0,     0],
[  0,   1,   0,     0],
[  0,   0,   1, 0.303],
[0.0, 0.0, 0.0,   1.0]])
```

The full transformation thous is the following:
```
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7
```

Which results in the following matrix:
```
simplify(T0_EE) = Matrix([
[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - (-sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*sin(q1)*sin(q4)*sin(q5) + 1.25*sin(q2)*cos(q1) - 0.303*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 0.054*sin(q2 + q3)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3) + 1.5*cos(q1)*cos(q2 + q3) + 0.35*cos(q1)],
[ ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3),  1.25*sin(q1)*sin(q2) - 0.303*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) - 0.054*sin(q1)*sin(q2 + q3) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3) + 1.5*sin(q1)*cos(q2 + q3) + 0.35*sin(q1) + 0.303*sin(q4)*sin(q5)*cos(q1)],
[                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                                                  (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3),                                     -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                                                                 -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
[                                                                                                                                                            0,                                                                                                                                                             0,                                                                                        0,                                                                                                                                                                                                            1.0]])
```
