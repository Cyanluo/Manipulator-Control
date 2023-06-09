#include "Population.h"
#include <iostream>
#include <algorithm>

namespace GeneticAlgorithm {

    Population::Population(unsigned long numberOfChromosome) 
	{
        this->chromosomeArray = new Chromosome*[numberOfChromosome];
        
		for (unsigned long i = 0; i < numberOfChromosome; i++) 
		{ // 新建立的数组，任意的 this->chromosomeArray[i] 未必是 0x00
            this->chromosomeArray[i] = nullptr;
        }
        
		this->numberOfChromosome = numberOfChromosome;
    }

    Population::~Population() 
	{
        for (unsigned long i = 0; i < this->numberOfChromosome; i++) 
		{
            if (nullptr != this->chromosomeArray[i]) 
			{
                delete this->chromosomeArray[i];
            }
        }

        delete[] this->chromosomeArray;
    }

    bool Population::setChromosome(unsigned long offset, Chromosome *chromosome) 
	{
        if (offset >= this->numberOfChromosome) 
		{
            return false;
        }
        
		if (nullptr == this->chromosomeArray[offset]) 
		{
            this->chromosomeArray[offset] = chromosome;
            this->isMaxFitnessChromosomeCache = false;
            return true;
        }
        
		Chromosome* origin = this->chromosomeArray[offset];
        
		if ((void*)origin == (void*)chromosome) 
		{
            return true;
        }
        
		if (this->isMaxFitnessChromosomeCache) 
		{
            if ((void*)(this->maxFitnessChromosomeCache) == (void*)chromosome) 
			{
                return false;
            }
            
			if (this->maxFitnessChromosomeCache->getFitness(target) > chromosome->getFitness(target)) 
			{
                this->isMaxFitnessChromosomeCache = false;
            } 
			else 
			{
                this->maxFitnessChromosomeCache = chromosome;
            }
        }
        
		this->chromosomeArray[offset] = chromosome;
        delete origin;
        return true;
    }

    bool Population::replaceChromosome(unsigned long offset, Chromosome *chromosome) 
	{
        return this->setChromosome(offset, chromosome);
    }

    Chromosome* Population::getChromosome(unsigned long offset)
	{
        if (offset >= this->numberOfChromosome) 
		{
            throw "Error, offset out of range, in \"Population::getChromosome\".";
        }
        
		if (!this->chromosomeArray[offset]) 
		{
            throw "Null pointer exception.";
        }
        
		return this->chromosomeArray[offset];
    }

    unsigned long Population::getSize() 
	{
        return this->numberOfChromosome;
    }

    void Population::sort() 
	{
		//cout << "number" << this->numberOfChromosome << endl;
		// cout << endl;
		// for(int i=0; i< this->numberOfChromosome; i++)
		// {
		// 	cout << i << ":" << this->chromosomeArray[i] << endl;
		// }
		// cout << endl;
		// for(int i=0; i < this->numberOfChromosome; i++)
		// {
		// 	cout << i << ":" << this->chromosomeArray[i]->getFitness() << endl;
		// }
		// cout << endl;
		// for(int i=0; i < this->numberOfChromosome; i++)
		// {
		// 	cout << i << ":" << this->chromosomeArray[i]->getFitness() << endl;
		// }
		// cout << endl;
        
		std::sort(
            &(this->chromosomeArray[0]),
            &(this->chromosomeArray[this->numberOfChromosome]),
            [this](Chromosome* a, Chromosome* b) -> bool {
		//		cout << "a:" << a << " " << "b:" << b << endl;
				if(b < (Chromosome*)0x10000 || a < (Chromosome*)0x10000)
					return false;
                return a->getFitness(target) > b->getFitness(target);
            }
        );
		// cout << endl;
		// for(int i=0; i< this->numberOfChromosome; i++)
		// {
		// 	cout << i << ":" << this->chromosomeArray[i] << endl;
		// }
		// cout << endl;
		// for(int i=0; i < this->numberOfChromosome; i++)
		// {
		// 	cout << i << ":" << this->chromosomeArray[i]->getFitness() << endl;
		// }
		// cout << endl;
        this->isMaxFitnessChromosomeCache = true;
        this->maxFitnessChromosomeCache = this->chromosomeArray[0];
        this->maxFitnessChromosomeOffset = 0;
    }

    Chromosome* Population::getMaxFitnessChromosome() 
	{
        if (this->isMaxFitnessChromosomeCache) 
		{
            return this->maxFitnessChromosomeCache;
        }
        
		long double maxFitness = std::numeric_limits<long double>::min();
        unsigned long offset = 0;
        this->maxFitnessChromosomeCache = this->chromosomeArray[offset];
        
		for (unsigned long i = 0; i < this->numberOfChromosome; i++) 
		{
            if (this->chromosomeArray[i]->getFitness(target) > maxFitness) 
			{
                offset = i;
                maxFitness = this->chromosomeArray[i]->getFitness(target);
            }
		}
        
		this->isMaxFitnessChromosomeCache = true;
        this->maxFitnessChromosomeCache = this->chromosomeArray[offset];
        this->maxFitnessChromosomeOffset = offset;
        
		return this->chromosomeArray[offset];
    }

	void Population::setTarget(TransferMatrix& target)
	{
		this->target = target;
	}

	void Population::resetMaxFitnessChCach()
	{
		this->isMaxFitnessChromosomeCache = false;
	}

	void Population::PSO(long double*& pbest,
						long double*& gbest,
						long double w,
						long double c1,
						long double c2,
						MatrixXd& limit)
	{		
		for (int i = 0; i < this->numberOfChromosome; i++) 
		{
            this->chromosomeArray[i]->PSO(pbest, gbest, w, c1, c2, limit);
		}
	}

}
