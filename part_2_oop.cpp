#include <iostream>
#include <math.h>
#include <cmath>
#include "include/CRoboticConstraints.cpp"
#include "include/CCartesian.cpp"
#include "include/CCommonUtils.cpp"
using namespace std;

int main(){
    float x, y, z;  // Input cartesian x, y, z coordinates from the user
    cout << "enter cartesian x: ";
    cin >> x;
    cout << "enter cartesian y: ";
    cin >> y;
    cout << "enter cartesian z: ";
    cin >> z;
    if (z <= 0)
        cout << "collision expected" << endl;  // if z <= 0 the robot will collide with the ground

    CCartesian cart;
    CCommonUtils cu;
    float t1,t2,t3;
    t1 = trunc(cart.Gett1(x,y,z)*float(180/M_PI));    // Calculate joint angle theta1 in degrees
    t3 = trunc(cart.Gett3(x,y,z)*float(180/M_PI));    // Calculate joint angle theta3 in degrees 
    t2 = trunc(cart.Gett2(x,y,z,0)*float(180/M_PI));  // Calculate joint angle theta2 in degrees
    if (isnan(t2)){                                   // theta2 equation was quadratic with two roots
        t2 = trunc(cart.Gett2(x,y,z,1)*float(180/M_PI));   // if theta2 is nan then check other root
        if (isnan(t2))                                     // if theta2 is still nan then it is 0
            t2 = 0;
        if (cu.IsInputInLimit(t1,t2,t3))                 // Are the joint values within specified limits?!
            cout << "One solution found:" << endl;
        else
            cout << "solution found but unreachable:" << endl;   // if they are not within limits then print "unreachable"
        cout << "t1:" << t1 << endl;                  // Print one solution as the other has theta2 as nan
        cout << "t2:" << t2 << endl;                  // All the printed joint angles are in degrees
        cout << "t3:" << t3 << endl;
    }
    else{
        cout << "Two solutions found. First solution:" << endl;  // if theta2 is not nan then both the solutions are valid and printed
        cout << "t1:" << t1 << endl;
        cout << "t2:" << t2 << endl;
        cout << "t3:" << t3 << endl;
        t2 = trunc(cart.Gett2(x,y,z,1)*float(180/M_PI));
        if (cu.IsInputInLimit(t1,t2,t3))                 // Are the joint values within specified limits?!
            cout << "Second solution:" << endl;
        else
            cout << "Second solution found but unreachable:" << endl;
        cout << "t1:" << t1 << endl;
        cout << "t2:" << t2 << endl;
        cout << "t3:" << t3 << endl;
    }
}