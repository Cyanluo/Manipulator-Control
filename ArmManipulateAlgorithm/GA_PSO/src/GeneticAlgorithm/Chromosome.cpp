#include <iostream>
#include "Chromosome.h"
#include "ChromosomeFactory.h"
#include "Utils/GlobalCppRandomEngine.h"

namespace GeneticAlgorithm {

    Chromosome::Chromosome(unsigned long lengthOfChromosome) 
	{
        this->dataArray = new long double[lengthOfChromosome];
        this->lengthOfData = lengthOfChromosome;

		Matrix<float, JOINTN, 4> dh;
		dh << -PI/2,   0,   2,   0,
		  		PI/2,   0,  0, PI/2,
		    	0,     10,   0,  0,
		    	0,     20,  0,   0,
		    	0,     30,   0,  0;
		
		TransferMatrix wf;

		arm = new DH_MechanicalArm<5, 5>(dh, wf);

		VectorXf runParams(5, 1);
		runParams << RADIAN(40),RADIAN(50),RADIAN(0),RADIAN(90),RADIAN(0);
		target = this->arm->forward(runParams);
    }

    Chromosome::~Chromosome() 
	{
        delete[] this->dataArray;
		delete arm;
    }

    bool Chromosome::setGene(unsigned long offset, long double value) {
        if (offset > this->lengthOfData - 1) {
            return false;
        }
        if (this->dataArray[offset] != value) {
            this->dataArray[offset] = value;
            this->isFitnessCached = false;
        }
        return true;
    }

    long double Chromosome::getGene(unsigned long offset) {
        if (offset > this->lengthOfData - 1) {
            throw "Error, out of range.";
        }
        return this->dataArray[offset];
    }

    void Chromosome::dump() 
	{
        cout << "Fitness=" << this->getFitness() << " ";
		for(unsigned long i = 0; i < this->lengthOfData; i++)
		{
			cout << this->dataArray[i] << " ";
		}
		cout << endl;
		cout << this->arm->forward() << endl;
		cout << "accuracy:" << sqrt((this->arm->forward() - target).squaredNorm()) << endl;
	}

    unsigned long Chromosome::getLength() {
        return this->lengthOfData;
    }

    long double Chromosome::getFitness() 
	{
		if (this->isFitnessCached) 
		{
            return this->fitnessCached;
        }
		
        if (this->lengthOfData < 2) 
		{
            throw "Can not less then 2.";
        }

		VectorXf runParams(5, 1);
		
		for(int i=0; i<5; i++)
		{
			runParams(i) = RADIAN(this->dataArray[i]);	
		}

		TransferMatrix ret;
		
		ret = this->arm->forward(runParams);
		
		ret = ret - this->target;
		for(int i=0; i<3; i++)
		{
			for(int j=0; j<3; j++)
			{
				ret(i, j) = ret(i, j) * 9;
			}
		}
		// cout <<  endl;
		// cout << runParams << endl;
		// cout << ret << endl;
		// cout << this->target << endl;
		
		this->fitnessCached = sqrt(ret.squaredNorm());
		// cout << this->fitnessCached << endl;
		// cout << endl;
		//cout << y << endl;
        // y 最小等于0，我们求最大适应度需要反过来
        this->fitnessCached = 1.0 / (this->fitnessCached + 0.01);

		this->isFitnessCached = true;

        return this->fitnessCached;
    }

    Chromosome* Chromosome::crossover(Chromosome* another) {
        if (another->getLength() != this->lengthOfData) {
            throw "Length not equals!";
        }
        long double* newData = new long double[this->lengthOfData];
        for (unsigned long i = 0; i < this->lengthOfData; i++) {
            newData[i] = (this->dataArray[i] + another->getGene(i)) / 2.0;
        }
        Chromosome* newChromosome = ChromosomeFactory().buildFromArray(newData, this->lengthOfData);
        delete[] newData;
        return newChromosome;
    }

    void Chromosome::mutation(long double r) {
        if (r <= 0.0) {
            return;
        }
        using GeneticAlgorithm::Utils::GlobalCppRandomEngine;
        std::normal_distribution<long double> distribution(0, r);
        for (unsigned long i = 0; i < this->lengthOfData; i++) {
            this->dataArray[i] += distribution(GlobalCppRandomEngine::engine);
        }

		this->isFitnessCached = false;
    }
}
