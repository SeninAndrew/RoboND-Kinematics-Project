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

