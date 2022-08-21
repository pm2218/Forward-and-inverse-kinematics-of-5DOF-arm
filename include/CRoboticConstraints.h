#ifndef CROBOTICCONTRAINTS_H_
#define CROBOTICCONTRAINTS_H_
#include<iostream>

using namespace std;

class CRoboticConstraints {
	public:
		CRoboticConstraints();
		virtual ~CRoboticConstraints();
		int a1 = 50;
		float l1 = 300;
		int l2 = 400;
		int l3 = 350;
		int l4 = 50;
};

#endif
