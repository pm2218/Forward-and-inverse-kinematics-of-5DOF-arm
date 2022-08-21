#ifndef CCARTESIAN_H_
#define CCARTESIAN_H_
#include<iostream>
#include "../eigen/Eigen/Dense"

using namespace std;

class CCartesian: public CRoboticConstraints {
    public:
        CCartesian();
        virtual ~CCartesian();
        float Gett1(float x, float y, float z);
        float Gett3(float x, float y, float z);
        float Gett2(float x, float y, float z, int temp);
        bool IsInputInLimit(int t1, int t2, int t3, int t4, int l5);
        Eigen::MatrixXf GetPose(Eigen::MatrixXf theta);
        Eigen::MatrixXf Jacobian(Eigen::MatrixXf theta);
    private:
        float t1,t2,t3;
};

#endif