#include <iostream>
#include <math.h>
#include <cmath>
#include "include/CCommonUtils.cpp"
#include "include/CRoboticConstraints.cpp"
#include "include/CMatrix.cpp"
using namespace std;

int main(){
    int t1, t2, t3;  // Input joint angles: theta1, theta2, theta3 respectively from the user
    float a, b, c; 
    cout << "enter theta1 in degrees: ";
    cin >> a;
    t1 = trunc(a);
    cout << "enter theta2 in degrees: ";
    cin >> b;
    t2 = trunc(b);
    cout << "enter theta3 in degrees: ";
    cin >> c;
    t3 = trunc(c);

    CCommonUtils cu;
    CMatrix matrix;
    if (cu.IsInputInLimit(t1,t2,t3)){  // Are the joint values within specified limits?!
        int t[3] = {abs(t1),abs(t2),abs(t3)};
        int max = cu.GetMax(t);  // getting the maximum value of the joint values
        int state[max] = {0};    
        float p,q,r;             // cartesian coordinates
        float H[4][4];           
        for(int k=0; k<=max; k++){  // State is determined at regular intervals between initial and final positions
            matrix.HomogTrans(t1,t2,t3,k,max,H);  // homogenous transformation of the end effector with respect to base
            r = matrix.GetElement(H,3,4);  // Get the cartesian z coordinate in the matrix
            if (r <= 0)            // state = 1 when z <= 0; state = 0 when z > 0
                state[k] = 1;
            else if (r == 0)
                state[k] = 0;
            else
                state[k] = 2;
        }
        cout << "remark:";
        if (cu.FindElement(state,max,1) == true)
            cout << "Collision" << endl;  // State = 1 indicates that robot will collide
        else
            cout << "Reachable" << endl;
        p = matrix.GetElement(H,1,4);  // Get the cartesian x coordinate from the matrix
        q = matrix.GetElement(H,2,4);  // Get the cartesian y coordinate from the matrix
        r = matrix.GetElement(H,3,4);  // Get the cartesian z coordinate from the matrix
        cout << "The cartesian coordinates are as follow:" << endl;
        cout << "x:" << p << endl;     // Print cartesian x, y, z coordinates in millimeter
        cout << "y:" << q << endl;
        cout << "z:" << r << endl;
    }
    else
        cout << "Unreachable" << endl; // joint values are not specified limits
    return 0;
}