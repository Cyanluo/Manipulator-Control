#ifndef MECHANICALARM_H_
#define MECHANICALARM_H_

#include"../eigen-3.3.9/Eigen/Dense"
#include"./TransferMatrix.h"

using namespace Eigen;

template< int JointN, int DriverJointN >
class JMechArm
{
protected:
	VectorXf RunParams;
	TransferMatrix* worldCoordinates;
	int m_runParamsLength;

	virtual void loadRunParams() = 0;
public:
	JMechArm(TransferMatrix& wf)
	{
		RunParams.setZero(DriverJointN, 1);

		worldCoordinates = new TransferMatrix(wf);

		m_runParamsLength = DriverJointN;
	}

	virtual TransferMatrix forward(VectorXf runParams) = 0;

	virtual TransferMatrix forward() = 0;

	virtual Vector3f calcPosture(TransferMatrix& t) = 0;
	
	virtual VectorXf backward(TransferMatrix& dst, bool update=false) = 0;

	VectorXf runParams() const
	{
		return RunParams;
	}

	int runParamsLength() const
	{
		return m_runParamsLength;
	}

	virtual ~JMechArm()
	{
		delete worldCoordinates;
	}
};

#endif //MECHANICALARM_H_