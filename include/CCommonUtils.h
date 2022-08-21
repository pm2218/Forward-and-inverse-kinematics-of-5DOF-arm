#ifndef CCOMMONUTILS_H_
#define CCOMMONUTILS_H_
#include<iostream>

using namespace std;

class CCommonUtils {
	public:
		CCommonUtils();
		virtual ~CCommonUtils();
		bool IsInputInLimit(int t1,int t2, int t3);
		int GetMax(int arr[3]);
		bool FindElement(int arr[], int size, int a);
};

#endif
