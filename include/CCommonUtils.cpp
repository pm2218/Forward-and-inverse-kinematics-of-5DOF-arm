#include "CCommonUtils.h"
#include <iostream>

using namespace std;

CCommonUtils::CCommonUtils() {
	// TODO Auto-generated constructor stub

}

CCommonUtils::~CCommonUtils() {
	// TODO Auto-generated destructor stub
}

bool CCommonUtils::IsInputInLimit(int t1,int t2, int t3){
	if ((-90 <= t1) && (t1 <= 90) && (-90 <= t2) && (t2 <= 90) && (-135 <= t3) && (t3 <= 135)){
		return true;
	}
	else
        return false;
}

int CCommonUtils::GetMax(int arr[3]){
    int i;
    int max = arr[0];
    for(i=0; i<3; i++){
        if(max < arr[i]){
            max = arr[i];
        }
    }
    return max;
}

bool CCommonUtils::FindElement(int arr[], int size, int a){
    int i;
    bool temp;
    for(i=0; i<size; i++){
        if(arr[i]==a){
            temp = true;
            break;
        }
    }
    return temp;
}
