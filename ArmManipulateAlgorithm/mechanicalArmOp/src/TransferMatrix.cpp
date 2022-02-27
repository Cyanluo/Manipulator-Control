#include"../inc/TransferMatrix.h"

TransferMatrix::TransferMatrix()
{
	this->setIdentity(4, 4);
}

TransferMatrix::~TransferMatrix()
{

}

TransferMatrix& TransferMatrix::operator -(const TransferMatrix& other)
{
	if(this != &other)
	{
		for(int i=0; i<this->cols(); i++)
		{
			this->col(i) -= other.col(i);
		}
	}

	return *this;
}
