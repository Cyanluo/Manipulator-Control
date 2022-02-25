#include "ChromosomeFactory.h"
#include "Chromosome.h"
#include <random>
#include "Utils/GlobalCppRandomEngine.h"

namespace GeneticAlgorithm {

    Chromosome* ChromosomeFactory::buildFromArray(long double data[], unsigned long lengthOfData) 
	{
        Chromosome* buildChromosome = this->buildEmpty(lengthOfData);
        
		for (unsigned long i = 0; i < lengthOfData; i++) 
		{
            if (!buildChromosome->setGene(i, data[i])) 
			{
                throw "Error, \"Chromosome::setGene\" return false. Exception throw in method \"ChromosomeFactory::buildFromArray\".";
            }
        }

        return buildChromosome;
    }

    Chromosome* ChromosomeFactory::buildRandomChromosome(unsigned long lengthOfData, MatrixXd& limit)
	{
        using namespace GeneticAlgorithm::Utils;
        long double *data = new long double[lengthOfData];
        
		vector<std::uniform_real_distribution<long double>*> array(limit.rows(), nullptr);

		for (unsigned long i = 0; i < lengthOfData; i++) 
		{
			array[i] = new std::uniform_real_distribution<long double>(limit(i, 0), limit(i, 1));
		}

		for (unsigned long i = 0; i < lengthOfData; i++) 
		{
			data[i] = (*(array[i]))(Utils::GlobalCppRandomEngine::engine);
		}

		for (unsigned long i = 0; i < lengthOfData; i++) 
		{
			delete array[i];
		}
		
        Chromosome* buildChromosome = this->buildFromArray(data, lengthOfData);
        delete[] data;

        return buildChromosome;
    }

    Chromosome* ChromosomeFactory::buildEmpty(unsigned long lengthOfData) 
	{
        return new Chromosome(lengthOfData);
    }

    Chromosome* ChromosomeFactory::buildFromChromosome(Chromosome* existsChromosome) 
	{
        Chromosome* result = this->buildEmpty(existsChromosome->getLength());
        
		for (unsigned long i = 0; i < result->getLength(); i++) 
		{
            result->setGene(i, existsChromosome->getGene(i));
        }
        
		return result;
    }

}
