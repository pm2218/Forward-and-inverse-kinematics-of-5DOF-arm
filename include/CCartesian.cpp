#include "CCartesian.h"
#include<iostream>
#include "../eigen/Eigen/Dense"
using namespace std;

CCartesian::CCartesian() {
	// TODO Auto-generated constructor stub

}

CCartesian::~CCartesian() {
	// TODO Auto-generated destructor stub
}

float CCartesian::Gett1(float x, float y, float z){
    t1 = (atan(y/x));
    return t1;
}

float CCartesian::Gett3(float x, float y, float z){
    float t2 = Gett1(x,y,z);
    t3 = (acos((pow(((x/cos(t1))-a1),2)+pow((z-l1),2)-pow(l2,2)-pow((l3+l4),2))/(2*l2*(l3+l4))));
    return t3;
}

float CCartesian::Gett2(float x, float y, float z, int temp){
    float t2 = Gett1(x,y,z);
    float t3 = Gett3(x,y,z);
    float a = ((z-l1)/((l3+l4)*sin(t3)));
    float b = ((l2+(l3+l4)*cos(t3))/((l3+l4)*sin(t3)));
    if (temp == 0)
        t2 = (acos(((2*a*b)+(pow(pow((2*a*b),2)-4*(pow(b,2)+1)*(pow(a,2)-1),0.5)))/(2*(pow(a,2)+1))));
    else if (temp == 1)
        t2 = (acos(((2*a*b)-(pow(pow((2*a*b),2)-4*(pow(b,2)+1)*(pow(a,2)-1),0.5)))/(2*(pow(a,2)+1))));
    return t2;
}

bool CCartesian::IsInputInLimit(int t1,int t2, int t3, int t4, int l5){
    if ((-90 <= t1) && (t1 <= 90) && (-90 <= t2) && (t2 <= 90) && (-135 <= t3) && (t3 <= 135) && (-135 <= t4) && (t4 <= 135) && (0 <= l5) && (l5 <= 50)){
        return true;
    }
    else
        return false;
}

Eigen::MatrixXf CCartesian::GetPose(Eigen::MatrixXf theta){
    Eigen::MatrixXf pose(3,1);
    pose(0,0) = cos(theta(0, 0))*(a1+l2*sin(theta(1, 0))+l3*sin(theta(1, 0)+theta(2, 0))-l4*sin(theta(1, 0)+theta(2, 0)+theta(3, 0)))+theta(4, 0)*sin(theta(0, 0));
    pose(1,0) = sin(theta(0, 0))*(a1+l2*sin(theta(1, 0))+l3*sin(theta(1, 0)+theta(2, 0))-l4*sin(theta(1, 0)+theta(2, 0)+theta(3, 0)))-theta(4, 0)*cos(theta(0, 0));
    pose(2,0) = l1+l2*cos(theta(1, 0))+l3*cos(theta(1, 0)+theta(2, 0))+l4*cos(theta(1, 0)+theta(2, 0)+theta(3, 0));
    return pose;
}

Eigen::MatrixXf CCartesian::Jacobian(Eigen::MatrixXf theta){
    Eigen::MatrixXf J(3,5);
    J(0,0) = -sin(theta(0, 0))*(a1+l2*sin(theta(1, 0))+l3*sin(theta(1, 0)+theta(2, 0))-l4*sin(theta(1, 0)+theta(2, 0)+theta(3, 0)))+theta(4, 0)*cos(theta(0, 0));
    J(1,0) = cos(theta(0, 0))*(a1+l2*sin(theta(1, 0))+l3*sin(theta(1, 0)+theta(2, 0))-l4*sin(theta(1, 0)+theta(2, 0)+theta(3, 0)))+theta(4, 0)*sin(theta(0, 0));
    J(2,0) = 0;

    J(0,1) = cos(theta(0, 0))*(l2*cos(theta(1, 0))+l3*cos(theta(1, 0)+theta(2, 0))-l4*cos(theta(1, 0)+theta(2, 0)+theta(3, 0)));
    J(1,1) = sin(theta(0, 0))*(l2*cos(theta(1, 0))+l3*cos(theta(1, 0)+theta(2, 0))-l4*cos(theta(1, 0)+theta(2, 0)+theta(3, 0)));
    J(2,1) = -l2*sin(theta(1, 0))-l3*sin(theta(1, 0)+theta(2, 0))-l4*sin(theta(1, 0)+theta(2, 0)+theta(3, 0));

    J(0,2) = cos(theta(0, 0))*(l3*cos(theta(1, 0)+theta(2, 0))-l4*cos(theta(1, 0)+theta(2, 0)+theta(3, 0)));
    J(1,2) = sin(theta(0, 0))*(l3*cos(theta(1, 0)+theta(2, 0))-l4*cos(theta(1, 0)+theta(2, 0)+theta(3, 0)));
    J(2,2) = -l3*sin(theta(1, 0)+theta(2, 0))-l4*sin(theta(1, 0)+theta(2, 0)+theta(3, 0));

    J(0,3) = -cos(theta(0, 0))*l4*cos(theta(1, 0)+theta(2, 0)+theta(3, 0));
    J(1,3) = -sin(theta(0, 0))*l4*cos(theta(1, 0)+theta(2, 0)+theta(3, 0));
    J(2,3) = -l4*sin(theta(1, 0)+theta(2, 0)+theta(3, 0));

    J(0,4) = sin(theta(0, 0));
    J(1,4) = -cos(theta(0, 0));
    J(2,4) = 0;
    return J;
}