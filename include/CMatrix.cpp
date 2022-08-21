#include "CMatrix.h"
#include<iostream>
using namespace std;

CMatrix::CMatrix() {
	// TODO Auto-generated constructor stub
} 

CMatrix::~CMatrix() {
	// TODO Auto-generated destructor stub
}

void CMatrix::Multiply(float mat1[4][4], float mat2[4][4], float res[4][4]) {
    int i,j,k;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            res[i][j] = 0;
            for (k = 0; k < 4; k++)
                res[i][j] += mat1[i][k] * mat2[k][j];
        }
    }
}

float CMatrix::GetElement(float mat[4][4], int x, int y){
    int i, j;
    float found;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            if (i == (x-1) && j == (y-1)){
                found = mat[i][j];
            }
        }
    }
    return found;
}

void CMatrix::HomogTrans(int t1, int t2, int t3, int k, int max, float H[4][4]){
    float H4[4][4] = {0};
    if (max != 0){
        float H1[4][4] = {{cos(float(k*t1/max)*float(3.14159265/180)), 0, sin(float(k*t1/max)*float(3.14159265/180)), a1*cos(float(k*t1/max)*float(3.14159265/180))},{sin(float(k*t1/max)*float(3.14159265/180)), 0, -cos(float(k*t1/max)*float(3.14159265/180)), a1*sin(float(k*t1/max)*float(3.14159265/180))},{0, 1, 0, l1},{0, 0, 0, 1}};
        float H2[4][4] = {{sin(float(k*t2/max)*float(3.14159265/180)),-cos(float(k*t2/max)*float(3.14159265/180)),0,l2*sin(float(k*t2/max)*float(3.14159265/180))},{cos(float(k*t2/max)*float(3.14159265/180)),sin(float(k*t2/max)*float(3.14159265/180)),0,l2*cos(float(k*t2/max)*float(3.14159265/180))},{0, 0, 1, 0},{0, 0, 0, 1}};
        float H3[4][4] = {{cos(float(k*t3/max)*float(3.14159265/180)),-sin(float(k*t3/max)*float(3.14159265/180)),0,(l3+l4)*cos(float(k*t3/max)*float(3.14159265/180))},{sin(float(k*t3/max)*float(3.14159265/180)),cos(float(k*t3/max)*float(3.14159265/180)),0,(l3+l4)*sin(float(k*t3/max)*float(3.14159265/180))},{0, 0, 1, 0},{0, 0, 0, 1}};
        Multiply(H1, H2, H4);
        Multiply(H4, H3, H);
    }
    else{
        float H1[4][4] = {{cos(float(t1)*float(3.14159265/180)), 0, sin(float(t1)*float(3.14159265/180)), a1*cos(float(t1)*float(3.14159265/180))},{sin(float(t1)*float(3.14159265/180)), 0, -cos(float(t1)*float(3.14159265/180)), a1*sin(float(t1)*float(3.14159265/180))},{0, 1, 0, l1},{0, 0, 0, 1}};
        float H2[4][4] = {{sin(float(t2)*float(3.14159265/180)),-cos(float(t2)*float(3.14159265/180)),0,l2*sin(float(t2)*float(3.14159265/180))},{cos(float(t2)*float(3.14159265/180)),sin(float(t2)*float(3.14159265/180)),0,l2*cos(float(t2)*float(3.14159265/180))},{0, 0, 1, 0},{0, 0, 0, 1}};
        float H3[4][4] = {{cos(float(t3)*float(3.14159265/180)),-sin(float(t3)*float(3.14159265/180)),0,(l3+l4)*cos(float(t3)*float(3.14159265/180))},{sin(float(t3)*float(3.14159265/180)),cos(float(t3)*float(3.14159265/180)),0,(l3+l4)*sin(float(t3)*float(3.14159265/180))},{0, 0, 1, 0},{0, 0, 0, 1}};
        Multiply(H1, H2, H4);
        Multiply(H4, H3, H);
    }
}