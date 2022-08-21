#ifndef CMATRIX_H_
#define CMATRIX_H_
#include<iostream>

using namespace std;

class CMatrix: public CRoboticConstraints {
    public:
        CMatrix();
        virtual ~CMatrix();
        void Multiply(float mat1[4][4], float mat2[4][4], float res[4][4]);
        float GetElement(float mat[4][4], int x, int y);
        void HomogTrans(int t1, int t2, int t3, int k, int max, float H[4][4]);
};

#endif