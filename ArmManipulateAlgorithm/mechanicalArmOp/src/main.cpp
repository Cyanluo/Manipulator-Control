#include"../inc/DH_MechanicalArm.h"
#include<math.h>
#include<iostream>
#include <iomanip>

using namespace std;
const double PI = 3.1415926;
#define RADIAN(a) (PI/180*a) 
const int JOINTN = 5;

int main()
{
	cout  << fixed << setprecision(1);

	Matrix<float, JOINTN, 4> dh;
	dh << -PI/2,   0,   2,   0,
		  PI/2,    0,   0,   PI/2,
		    0,     10,   0,  0,
		    0,     20,  0,   0,
		    0,     30,   0,  0;

	TransferMatrix wf;

	DH_MechanicalArm<5, 5> arm(dh, wf);

	VectorXf runParams(JOINTN, 1);
	runParams << RADIAN(40),RADIAN(50),RADIAN(0),RADIAN(90),RADIAN(0);

	TransferMatrix ret = arm.forward(runParams);

	cout << JOINTN << "->" << "w" << endl << ret << endl;	

	return 0;
}
