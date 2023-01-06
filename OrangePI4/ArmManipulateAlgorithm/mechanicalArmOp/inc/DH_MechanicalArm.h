#ifndef DHMECHANICALARM_H_
#define DHMECHANICALARM_H_

#include "MechanicalArm.h"
#include <math.h>
#include <iostream>

using namespace std;

#define R2ANGEL(a) (180/PI*(a));

template< int JointN, int DriverJointN >
class DH_MechanicalArm:public JMechArm<JointN, DriverJointN>
{
	typedef Matrix<float, JointN, 4> MatrixN4;
	MatrixN4 DHTable;
	const double PI = 3.1415926;
protected:
	void loadRunParams()
	{
		// this->RunParams.conservativeResize(DriverJointN + 1, 1);

		// this->RunParams(DriverJointN) = this->RunParams(DriverJointN-1);
		// this->RunParams(DriverJointN-1) = 0;
		DHTable.col(3) += this->RunParams;		
	}

	TransferMatrix i2i_1TransferMatrix(int i, bool& success)
	{
		TransferMatrix ret;
		
		success = false;
		i -= 1;

		if( (0<=i) && (i<JointN) )
		{// af:DHTable(i, 0) a:DHTable(i, 1) d:DHTable(i, 2) st:DHTable(i, 3)
			ret(0, 0) = cos(DHTable(i, 3)); // cos(st~i~)
			ret(0, 1) = -sin(DHTable(i, 3));// -sin(st~i~)
			ret(0, 3) = DHTable(i, 1);      // a~i-1~
			ret(1, 0) = sin(DHTable(i, 3)) * cos(DHTable(i, 0)); // sin(st~i~)*cos(af~i-1~)
			ret(1, 1) = cos(DHTable(i, 3)) * cos(DHTable(i, 0));
			ret(1, 2) = -sin(DHTable(i, 0));
			ret(1, 3) = -sin(DHTable(i, 0)) * DHTable(i, 2);
			ret(2, 0) = sin(DHTable(i, 3)) * sin(DHTable(i, 0));
			ret(2, 1) = cos(DHTable(i, 3)) * sin(DHTable(i, 0));
			ret(2, 2) = cos(DHTable(i, 0));
			ret(2, 3) = cos(DHTable(i, 0)) * DHTable(i, 2);
			
			success = true;
		}

		return ret;
	}

public:
	DH_MechanicalArm(TransferMatrix& wf):JMechArm<JointN, DriverJointN>(wf)
	{
		DHTable.setZero(JointN, 4);  
	}

	DH_MechanicalArm(MatrixN4& dh, TransferMatrix& wf): JMechArm<JointN, DriverJointN>(wf)
	{
		DHTable = MatrixN4(dh);
	}

	TransferMatrix forward(VectorXf runParams)
	{
		TransferMatrix ret(*(this->worldCoordinates));

		TransferMatrix temp;

		bool s = true;
		
		this->RunParams = runParams;
		loadRunParams();
		 //std::cout << "dh:" << std::endl  << DHTable << std::endl;
		// std::cout << "0->w" << std::endl  << ret << std::endl;
		for(int i = 1; i <=JointN && s; i++)
		{
			temp = i2i_1TransferMatrix(i, s);
           // std::cout << i << "->" << i-1 << std::endl << temp << std::endl;
			ret *= temp;
		}

        DHTable.col(3) -= this->RunParams;

        return ret;
	}

	TransferMatrix forward()
	{
		return forward(this->RunParams);
	}

	Vector3f calcPosture(TransferMatrix& t)
	{
		Vector3f pos(3);
		//b
		pos(1) = R2ANGEL(atan2(-t(2, 0), sqrt(t(0, 0)*t(0, 0)+t(1, 0)*t(1, 0))));
		
		if( 90 == int(pos(1)) )
		{
			pos(2) = 0;
			pos(0) = R2ANGEL(atan2(t(0, 1), t(1, 1)));
		}
		else if(-90 == int(pos(1)) )
		{
			pos(2) = 0;
			pos(0) = -R2ANGEL(atan2(t(0, 1), t(1, 1)));
		}
		else
		{
			//a
			pos(2) = R2ANGEL(atan2(t(1, 0)/cos(pos(1)), t(0, 0)/cos(pos(1))));
			//y
			pos(0) = R2ANGEL(atan2(t(2, 1)/cos(pos(1)), t(2, 2)/cos(pos(1))));
		}
		
		return pos;
	}

	VectorXf backward(TransferMatrix& dst, bool update=false)
	{
		
	}

	~DH_MechanicalArm()
	{
		
	}
};

#endif // DHMECHANICALARM_H
