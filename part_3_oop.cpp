#include <iostream>
#include <cmath>
#include "eigen/Eigen/Dense"
#include "include/CRoboticConstraints.cpp"
#include "include/CCartesian.cpp"
using namespace std;

int main(){
    CRoboticConstraints robot;
    CCartesian cu;
    CCartesian cart;

    Eigen::MatrixXf e(3, 1);  // error between current pose and input pose
    Eigen::MatrixXf Xd(3, 1); // Input cartesian x, y, z coordinates from the user

    cout << "enter cartesian x: ";
    cin >> Xd(0,0);
    cout << "enter cartesian y: ";
    cin >> Xd(1,0);
    cout << "enter cartesian z: ";
    cin >> Xd(2,0);
    if (Xd(2,0) <= 0)
        cout << "collision expected" << endl;  // if z <= 0 the robot will collide with the ground
  
    Eigen::MatrixXf i_1_theta(5, 1);  // temporary joint angles variables in every iteration
    Eigen::MatrixXf i_theta(5, 1);    // final joint angles

    Eigen::MatrixXf pose(3,1);        // temporary pose variables in every iteration
    Eigen::MatrixXf J(3,5);           // Jacobian

    i_theta(0, 0) = 0;                // Initial guess
    i_theta(1, 0) = 0;
    i_theta(2, 0) = 0;
    i_theta(3, 0) = 0;
    i_theta(4, 0) = 0;

    pose = cu.GetPose(i_theta);       // calculate pose for initial guess

    e = Xd - pose;                    // error between current and input pose

    while ((abs(e(0, 0)) > 0.001) || (abs(e(1, 0)) > 0.001) || (abs(e(2, 0)) > 0.001)){

        J = cu.Jacobian(i_theta);     // find jacobian for current joint angles

        Eigen::MatrixXf JPsedoInv = J.completeOrthogonalDecomposition().pseudoInverse();  // find pseudo inverse of Jacobian
        i_1_theta = i_theta + JPsedoInv * e;   // get new joint angles by compensating the error

        pose = cu.GetPose(i_1_theta);   // calculate pose for current angles

        e = Xd - pose;                  // new error
        i_theta = i_1_theta;            // new joint angles
    }

    //      Solutions do converge but they are often outside joint limits.                      //
    //      The below method solves this problem to some extent but is not completely correct   //

    for (int i = 0; i < 4; i++){         // process the final joint angles such that they are within specified limits
        if (i_theta(i,0) > 0)
            while (i_theta(i,0) > M_PI)
                i_theta(i,0) = i_theta(i,0) - M_PI;
        else if (i_theta(i,0) < 0)
            while (i_theta(i,0) < -M_PI)
                i_theta(i,0) = M_PI + i_theta(i,0);
        i_theta(i,0) = int(i_theta(i,0)*float(180/M_PI));
    }
    if (cart.IsInputInLimit(int(i_theta(0,0)),int(i_theta(1,0)),int(i_theta(2,0)),int(i_theta(3,0)),int(i_theta(4,0))))  // Are the joint values within specified limits?!
        cout << "Solution reachable:" << endl;
    else
        cout << "solution unreachable:" << endl;
    cout << i_theta << endl;
}