#include <iostream>
#include "Chromosome.h"
#include "ChromosomeFactory.h"
#include "Utils/GlobalCppRandomEngine.h"
#include <random>

namespace GeneticAlgorithm {

    Chromosome::Chromosome(unsigned long lengthOfChromosome) 
	{
        this->dataArray = new long double[lengthOfChromosome];
		this->velocity = new long double[lengthOfChromosome];
        
		this->lengthOfData = lengthOfChromosome;

		Matrix<float, JOINTN, 4> dh;
        dh << -PI/2,   0,   0,   0,
                PI/2,   0,  0,   PI/2,
                PI,     105,   0,  0,
                0,     98,  0,   0,
                0,     150,   0,  0;
//        dh << -PI/2,   0,   0,   0,
//               PI/2,   0,  0,   PI/2,
//                PI,     105,   0,  0,
//                0,     98,     0,   0,
//              -PI/2,     20,   40,  0,
//                PI/2,    110,   0,  0;
		
		TransferMatrix wf;

        arm = new DH_MechanicalArm<JOINTN, JOINTN>(dh, wf);
    }

    Chromosome::~Chromosome() 
	{
        delete[] this->dataArray;
		delete[] this->velocity;
		delete arm;
    }

    bool Chromosome::setGene(unsigned long offset, long double value) 
	{
        if (offset > this->lengthOfData - 1) 
		{
            return false;
        }
        
		if (this->dataArray[offset] != value) 
		{
            this->dataArray[offset] = value;
            this->isFitnessCached = false;
        }
        
		return true;
    }

	bool Chromosome::setVelocity(unsigned long offset, long double value)
	{
		if (offset > this->lengthOfData - 1) 
		{
            return false;
        }
        
		if (this->velocity[offset] != value) 
		{
            this->velocity[offset] = value;
        }
        
		return true;
	}


    long double Chromosome::getGene(unsigned long offset) 
	{
        if (offset > this->lengthOfData - 1) 
		{
            throw "Error, out of range.";
        }

        return this->dataArray[offset];
    }

	long double Chromosome::getVelocity(unsigned long offset)
	{
		if (offset > this->lengthOfData - 1) 
		{
            throw "Error, out of range.";
        }

        return this->velocity[offset];
	}

    void Chromosome::dump() 
	{
        cout << "Fitness=" << this->getFitness(target) << " ";
		
		for(unsigned long i = 0; i < this->lengthOfData; i++)
		{
			cout << this->dataArray[i] << " ";
		}
		cout << endl;

		for(unsigned long i = 0; i < this->lengthOfData; i++)
		{
			cout << this->velocity[i] << " ";
		}
		cout << endl;

		double postionInaccuracy = 0;
		double postureInaccuracy = 0;
		TransferMatrix ret = this->arm->forward();
		calcInaccuracy(ret, target, postionInaccuracy, postureInaccuracy);
		cout << ret << endl;
		cout << "postionInaccuracy:" << postionInaccuracy << " " << "postureInaccuracy:" << postureInaccuracy << endl;
		//cout << this->arm->forward() << endl;
        cout << "inaccuracy:" << this->inaccuracy << endl;
	}

    unsigned long Chromosome::getLength() 
	{
        return this->lengthOfData;
    }

	void Chromosome::calcInaccuracy(TransferMatrix& l,
						 TransferMatrix& target,
						 double& postionInaccuracy,
						 double& postureInaccuracy)
	{
		Vector3f pl = this->arm->calcPosture(l);
		Vector3f pt = this->arm->calcPosture(target);

		postureInaccuracy = sqrt((pl - pt).squaredNorm());
		postionInaccuracy = sqrt((l.col(3).head(3) - target.col(3).head(3)).squaredNorm());
	}

    long double Chromosome::getFitness(TransferMatrix& target) 
	{
		if (this->isFitnessCached) 
		{
            return this->fitnessCached;
        }
		
        if (this->lengthOfData < 2) 
		{
            throw "Can not less then 2.";
        }

        VectorXf runParams(JOINTN, 1);
		
        for(int i=0; i<JOINTN; i++)
		{
			runParams(i) = RADIAN(this->dataArray[i]);	
		}

		TransferMatrix ret;
		
		ret = this->arm->forward(runParams);
		
		this->target = target;

		double postionInaccuracy = 0;
		double postureInaccuracy = 0;

        calcInaccuracy(ret, target, postionInaccuracy, postureInaccuracy);
//        ret = ret - this->target;
//        for(int i=0; i<3; i++)
//        {
//            for(int j=0; j<3; j++)
//            {
//                ret(i, j) = ret(i, j) * 10;
//            }
//        }
        this->inaccuracy = postionInaccuracy;
        //this->fitnessCached = sqrt(ret.squaredNorm());
        //this->inaccuracy = sqrt((this->arm->forward() - this->target).squaredNorm());
        this->fitnessCached = postionInaccuracy;
		//cout << this->fitnessCached << endl;
        // y 最小等于0，我们求最大适应度需要反过来
        this->fitnessCached = 1.0 / (this->fitnessCached/40 + 0.01);

		this->isFitnessCached = true;

        return this->fitnessCached;
    }

	double Chromosome::getInaccuracy()
	{
		return this->inaccuracy;
	}

    VectorXf Chromosome::getResult()
    {
        VectorXf runParams(JOINTN, 1);

        for(int i=0; i<JOINTN; i++)
        {
            runParams(i) = RADIAN(this->dataArray[i]);
        }

        return runParams;
    }

	void Chromosome::limiting(long double*& data, MatrixXd& limit)
	{
		for(int i=0; i<this->lengthOfData; i++)
		{
			if(data[i] > limit(i, 1))
			{	
				data[i] = limit(i, 1);
			}
			else if(data[i] < limit(i, 0))
			{
				data[i] = limit(i, 0);
			}
			else
			{
				continue;
			}
		}
	}

	// 限制速度大小，不改变方向
	void Chromosome::limiting(long double*& data, long double min, long double max)
	{
		long double v = 0.0;

		for(int i=0; i<this->lengthOfData; i++)
		{
			v += data[i] * data[i];
		}
		
		if( (-0.000001 < v) && (v < 0.000001) )
		{
			return;
		}

		v = sqrt(v);

		if( (min <= v) && (v <= max) )
		{
			return;
		}
		else if(v < min)
		{
			for(int i=0; i<this->lengthOfData; i++)
			{
				data[i] = data[i] / v * min;
			}
		}
		else
		{
			for(int i=0; i<this->lengthOfData; i++)
			{
				data[i] = data[i] / v * max;
			}
		}
	}

	void Chromosome::limitV(long double min, long double max)
	{
		limiting(this->velocity, min, max);
	}

    Chromosome* Chromosome::crossover(Chromosome* another, MatrixXd& limit) 
	{
        if (another->getLength() != this->lengthOfData) 
		{
            throw "Length not equals!";
        }
        
		long double* newData = new long double[this->lengthOfData];
		long double* newVelocity = new long double[this->lengthOfData];
        
		for (unsigned long i = 0; i < this->lengthOfData; i++) 
		{
            newData[i] = (this->dataArray[i] + another->getGene(i)) / 2.0;
            newVelocity[i] = (this->velocity[i] + another->getVelocity(i)) / 2.0;
            //newVelocity[i] = this->velocity[i];
        }

		limiting(newData, limit);
        limiting(newVelocity, limit(this->lengthOfData, 0), limit(this->lengthOfData, 1));

        Chromosome* newChromosome = ChromosomeFactory().buildFromArray(newData, newVelocity, this->lengthOfData);
        
		delete[] newData;
		delete[] newVelocity;
        
		return newChromosome;
    }

    void Chromosome::mutation(long double r, MatrixXd& limit) 
	{
        if (r <= 0.0) 
		{
            return;
        }
		
        using GeneticAlgorithm::Utils::GlobalCppRandomEngine;
        std::normal_distribution<long double> distribution(0, r);
        
		for (unsigned long i = 0; i < this->lengthOfData; i++) 
		{
            this->dataArray[i] += distribution(GlobalCppRandomEngine::engine);
        }

		limiting(this->dataArray, limit);
		
		this->isFitnessCached = false;
    }

	void Chromosome::arrayMUL(long double*& dst, long double*& l, long double c)
	{
		if(dst != nullptr)
		{
			for(int i=0; i<this->lengthOfData; i++)
			{
				dst[i] = l[i] * c;
			}
		}
	}

	void Chromosome::arraySUB(long double*& dst, long double*& l, long double*& r)
	{
		if(dst != nullptr)
		{
			for(int i=0; i<this->lengthOfData; i++)
			{
				dst[i] = l[i] - r[i];
			}
		}
	}

	void Chromosome::arrayADD(long double*& dst, long double*& l, long double*& r)
	{
		if(dst != nullptr)
		{
			for(int i=0; i<this->lengthOfData; i++)
			{
				dst[i] = l[i] + r[i];
			}
		}
	}

	void Chromosome::arrayCOPY(long double*& src, long double*& dst)
	{
		for(int i=0; i<this->lengthOfData; i++)
		{
			dst[i] = src[i];
		}
	}

	void Chromosome::PSO(long double*& pbest,
						long double*& gbest,
						long double w,
						long double c1,
						long double c2,
						MatrixXd& limit)
	{
		long double* ptemp = new long double[this->lengthOfData];
		long double* gtemp = new long double[this->lengthOfData];
		long double* vtemp = new long double[this->lengthOfData];

		arraySUB(ptemp, pbest, this->dataArray);
		arraySUB(gtemp, gbest, this->dataArray);

		std::uniform_real_distribution<long double> engine(0, 1);

		arrayMUL(ptemp, ptemp, c1*engine(Utils::GlobalCppRandomEngine::engine));
		arrayMUL(gtemp, gtemp, c2*engine(Utils::GlobalCppRandomEngine::engine));
		//arrayMUL(ptemp, ptemp, c1);
		//arrayMUL(gtemp, gtemp, c2);
		arrayMUL(vtemp, this->velocity, w);

		arrayADD(ptemp, ptemp, gtemp);
		arrayADD(this->velocity, ptemp, vtemp);
		
		limitV(limit(this->lengthOfData, 0), limit(this->lengthOfData, 1));

		arrayADD(this->dataArray, this->dataArray, this->velocity);

		limiting(this->dataArray, limit);
		
		this->isFitnessCached = false;

		delete[] ptemp;
		delete[] gtemp;
		delete[] vtemp;
	}

	void Chromosome::getData(long double*& dst)
	{
		arrayCOPY(this->dataArray, dst);
	}
}
