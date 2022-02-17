#ifndef TRANSFERMATRIX_H_
#define TRANSFERMATRIX_H_

#include"../eigen-3.3.9/Eigen/Dense"

using namespace Eigen;

class TransferMatrix:public Matrix4f
{
public:
	TransferMatrix();
	~TransferMatrix();
};

#endif // TRANSFERMATRIX_H_