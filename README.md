# Welcome to my 4DOF Pick and Sort Robotic Manipulator repository

It is recommended to add the src file to your MATLAB path and then develop code for each lab in a separate folder (lab1, lab2, etc.). This way you can modify your Robot class without needing to copy it over to each new lab folder.

<img width="560" height="420" alt="fullTriangleVelocities" src="https://github.com/user-attachments/assets/c9290a6f-0854-4d3f-afc8-ac875d2d8cc6" />

<img width="1214" height="674" alt="Lab4TargetAndRealVelocitiesGraphed" src="https://github.com/user-attachments/assets/aa0285f1-dcfd-4892-9a92-35c7df530a2f" />

---

## Dummy Mode

To test FK/IK without having to connect to the robot, create your robot object like this:

```matlab
robot = Robot(true);  % dummy mode enabled
% robot = Robot();    % dummy mode disabled by default
```

See example usage at [test_dh2fk.m](src/Lab2/test_dh2fk.m), [test_dh2mat.m](src/Lab2/test_dh2mat.m), [test_fk3001.m](src/Lab2/test_fk3001.m)

---

## IK Autograder

To test your IK implementation, run the autograder (`checkIK.p`) from the `src` folder in the MATLAB:
1. Either run `checkIK` from the MATLAB command window or
2.  Right-click on `checkIK.p` and select "Run"

You should see a message like this:
```
Testing ik3001 for [x, y, z, gamma]=[250.000000 250.000000 200.000000 6.000000]
	Pass (0.106030) < 1 mm
Testing ik3001 for [x, y, z, gamma]=[250.000000 250.000000 200.000000 7.000000]
	Pass (0.106143) < 1 mm
Testing ik3001 for [x, y, z, gamma]=[250.000000 250.000000 200.000000 8.000000]
	Pass (0.106195) < 1 mm
Testing ik3001 for [x, y, z, gamma]=[250.000000 250.000000 200.000000 9.000000]
	Fail (305.772498) > 1 mm
Testing ik3001 for [x, y, z, gamma]=[250.000000 250.000000 200.000000 10.000000]
	Fail (305.772498) > 1 mm
Total tests: 607257, Successful tests: 606127, Success Rate: 0.998139
```

⚠️ Note 1: The autograder assumes that your ik3001 implementation
- expects an **input** of 1x4 matrix [ $p_x$, $p_y$, $p_z$, $\gamma$], where $p_x$, $p_y$, and $p_z$ are in millimeters, and $\gamma$ is in **degrees**.
- and produces an **output** of 1x4 matrix [ $\theta_1$, $\theta_2$, $\theta_3$, $\theta_4$], where all values are in **degrees**.

⚠️ Note 2: The autograder uniformly sweeps across a range of test points in all four dimensions ($p_x$, $p_y$, $p_z$, $\gamma$) to make sure correct joint angles (with a +- 1 mm end-effector tolerance) are being returned.
Some of these points may be impossible to achieve, so don't shoot for a 100% success rate!

⚠️ Note 3: As you begin to develop your IK implementation, you might first want to pick a few test points to manually check that your code is working correctly.
You should create a separate MATLAB script for this purpose, for example at `src/Lab3/test_ik3001.m`, and enable the [dummy mode](#dummy-mode) to avoid having to connect to the robot.
**The autograder isn't really designed to help you debug your code**, so even a simple sign mistake can cause the entire test suite to fail!

---

## DK Autograder

To test your DK implementation, run the autograder (`checkDK.p`) from the `src` folder in the MATLAB:
1. Either run `checkDK` from the MATLAB command window or
2.  Right-click on `checkDK.p` and select "Run"

You should see a message like this:
```
Running the autograder...
	Done.Total tests: 6561, Successful tests: 6561, Success Rate: 1.000000
```

⚠️ Note 1: The autograder assumes that your dk3001 implementation
- expects an **input** of two 1x4 matrices [ $\theta_1$, $\theta_2$, $\theta_3$, $\theta_4$] and [ $\dot{\theta_1}$, $\dot{\theta_2}$, $\dot{\theta_3}$, $\dot{\theta_4}$], where all angles are in **degrees**, and all angular velocities in **degrees/second**.
- and produces an **output** of 6x1 matrix [ $v_x$, $v_y$, $v_z$, $\dot{\phi}$, $\dot{\theta}$, $\dot{\psi}$], where all velocities are in **mm/second**, and all orientations are in **degrees**.

⚠️ Note 2: The autograder uniformly sweeps across a range of test points in all eight dimensions ($\theta_1$, $\theta_2$, $\theta_3$, $\theta_4$, $\dot{\theta_1}$, $\dot{\theta_2}$, $\dot{\theta_3}$, $\dot{\theta_4}$) to make sure correct linear/angular velocities (with a +- 1 deg or deg/sec end-effector tolerance) are being returned.

⚠️ Note 3: As you begin to develop your DK implementation, you might first want to pick a few test points to manually check that your code is working correctly.
You should create a separate MATLAB script for this purpose, for example at `src/Lab4/test_dk3001.m`, and enable the [dummy mode](#dummy-mode) to avoid having to connect to the robot.
**The autograder isn't really designed to help you debug your code**, so even a simple sign mistake can cause the entire test suite to fail!

⚠️ Note 4: All the test points and their pass/fail status will be saved to a file named `AutograderDK3001Results.txt` in `src/`

---

## Extra resources

- https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/

- https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/

- https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/

