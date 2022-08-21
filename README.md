# Forward-and-inverse-kinematics-of-5DOF-arm

The project is divided into 3 parts/files based on the details specified in "Tasks.pdf":

1. **part_1_oop.cpp** - Forward kinematics of a 3DOF robotic arm that receives three joint angle inputs and outputs Cartesian coordinates in the x, y, and z axes. Additionally, it specifies if there will be any ground collisions or if the joint angles are outside of the allowed range.
2. **part_2_oop.cpp** - Analytical Inverse Kinematics of 3DOF arm, which accepts the Cartesian coordinates of x, y, and z and returns one or two sets of potential joint angle values. Adding up any collisions and checking to see if the joint angles fall outside of the specified range.
3. **part_3_oop.cpp** - Iterative Inverse Kinematics of 5DOF arm that uses the Newton-Raphson method to determine the joint angles The user provides the x, y, and z Cartesian coordinates. 

The folder also contains the following files:

1. **DH.pdf** - This file contains all the necessary handwritten calculations, such as calculating the Jacobian and Denavit Hartenberg matrices for the robotic arm.
2. **test.xlsx** - Some testing cases of the 3DOF robotic arm.
3. **include folder** - This folder contains all the header and source files of the classes.
4. **Tasks.pdf** - This file contains details regarding the three components mentioned above as well as constraints for a robotic arm.
5. **.exe files** - Build files of the three cpp files mentioned above.

Visual studio code is used to write, build and run the code. Eigen library is used for matrix operations. The links to install visual studio and directions to include Eigen are mentioned below:

**Visual studio code** - https://code.visualstudio.com/docs/languages/cpp

**Eigen library** - 
1. Download the zip file containing all the header and source files from this website - https://eigen.tuxfamily.org/index.php?title=Main_Page
2. Extract the zip file and move the folder to the current directory (containing the above files).
3. Rename the eigen folder to "eigen".

**References:**
1. https://www.rosroboticslearning.com/inverse-kinematics
2. https://www.rosroboticslearning.com/jacobian
