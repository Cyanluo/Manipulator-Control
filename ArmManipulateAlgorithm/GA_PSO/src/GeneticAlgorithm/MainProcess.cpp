#include <random>
#include <iostream>
#include "MainProcess.h"
#include "PopulationFactory.h"
#include "Utils/GlobalCppRandomEngine.h"
#include "Chromosome.h"

namespace GeneticAlgorithm {

    MainProcess::MainProcess() 
	{
		this->newChromosome = nullptr;
		this->selectedChromosome = nullptr;
		this->populations = nullptr;
    }

    MainProcess::~MainProcess() 
	{
        this->freeMemory();
    }

    void MainProcess::run(
		unsigned long numberOfPopulation,
        unsigned long numberOfChromosome,
        unsigned long lengthOfChromosome,
        MatrixXd& limit,
        unsigned long maxLoop,
        long double stopFitness,
        unsigned long keep,
        long double r,
		TransferMatrix& target
    ) 
	{	
        using namespace std;
    	this->freeMemory(); // 防止重复调用run()没有释放上一次的内存    
		this->numberOfPopulation = numberOfPopulation;
        this->numberOfChromosome = numberOfChromosome;
        this->lengthOfChromosome = lengthOfChromosome;
        this->keep = keep;
        this->kill = numberOfChromosome - keep;
        this->r = r;
		this->target = target;
		this->limit = limit;
        this->init(numberOfPopulation);
		
		for(int i=0; i<this->numberOfPopulation; i++)
		{
			this->populations[i]->setTarget(this->target);
		}
        
		this->sort();
		

		int index = 0;
		getMaxFitness(index);
		
        if (this->debug) 
		{
            cout << endl;
			cout << "代数=0, 最大适应度=" << this->maxFitness << ", 个体信息=";
			this->populations[index]->getMaxFitnessChromosome()->dump();
		}

        while (this->loopNow < maxLoop && this->maxFitness < stopFitness) 
		{
            this->select();
            this->crossover();
			// 变异率自适应
			this->pm = (this->r)*(1-(double(this->loopNow)/maxLoop));
            this->mutation();
            this->generated();
            this->sort();           
			this->loopNow++;
            			
			if (this->debug) 
			{
				getMaxFitness(index);
                cout << "代数=" << this->loopNow << ", 最大适应度=" << this->maxFitness << ", 个体信息=";
				this->populations[index]->getMaxFitnessChromosome()->dump();
            }
        }
		
        if (this->debug) 
		{
            cout << "结束。" << endl;
        }
    }

    void MainProcess::setDebug(bool enableDebug) 
	{
        this->debug = enableDebug;
    }

    unsigned long MainProcess::getLoopNumber() 
	{
        return this->loopNow;
    }
 
    long double MainProcess::getMaxFitness() 
	{
        return this->maxFitness;
    }

	long double MainProcess::getMaxFitness(int& indexPopulation)
	{
		double ret = 0;

		indexPopulation = 0;
		this->maxFitness = 0;

		for(int i=0; i<this->numberOfPopulation; i++)
		{
        	ret = this->populations[i]->getMaxFitnessChromosome()->getFitness(this->target);
		
			if(ret > this->maxFitness)
			{
				this->maxFitness = ret;
				indexPopulation = i;
			}
		}

		return ret;
	}

    Chromosome* MainProcess::getMaxFitnessChromosome() 
	{
		int index = 0;
		getMaxFitness(index);
        
		return this->populations[index]->getMaxFitnessChromosome();
    }

    // void MainProcess::replaceChromosome(Chromosome* chromosome) 
	// {
    //     Chromosome* maxChromosome = this->population->getMaxFitnessChromosome();
        
	// 	for (unsigned long offset = this->numberOfChromosome - 1; offset + 2 > 1; offset--) 
	// 	{
    //         if ((void*)this->population->getChromosome(offset) != (void*)maxChromosome) 
	// 		{
    //             this->population->replaceChromosome(offset, chromosome);
                
	// 			return;
    //         }
    //     }
    // }

    void MainProcess::init(unsigned long numberOfPopulation) 
	{
		this->populations = new Population*[numberOfPopulation];
		this->selectedChromosome = new Chromosome**[numberOfPopulation];
		this->newChromosome = new Chromosome**[numberOfPopulation];
		
		for(int i=0; i<this->numberOfPopulation; i++)
		{
			this->populations[i] = PopulationFactory().buildRandomPopulation(this->numberOfChromosome, this->lengthOfChromosome, this->limit);
		}
        
        this->loopNow = 0;
        this->maxFitness = 0.0;

		for(int i=0; i<this->numberOfPopulation; i++)
		{
			this->selectedChromosome[i] = new Chromosome*[2 * this->kill];
        	this->newChromosome[i] = new Chromosome*[this->kill];			
		}     
    }

    void MainProcess::sort() 
	{
        if (1 != this->keep) 
		{ // 为了优化流程，只保留一个的时候不必排序
			for(int i=0; i<this->numberOfPopulation; i++)
			{
				this->populations[i]->sort();
			}
        }
    }

    // 锦标赛选择算法，执行速度比轮盘赌的快
    void MainProcess::select() 
	{
        using namespace GeneticAlgorithm::Utils;
        using namespace std;
        Chromosome* selectChromosome1;
        Chromosome* selectChromosome2;
        unsigned long generate = 2 * this->kill;
        uniform_int_distribution<unsigned long> range(0, this->numberOfChromosome - 1);
        
		// 运行 generate 次选择
		for(int i=0; i<this->numberOfPopulation; i++)
		{
			for (unsigned long j = 0; j < generate; j++) 
			{
				selectChromosome1 = this->populations[i]->getChromosome(range(GlobalCppRandomEngine::engine));
				selectChromosome2 = this->populations[i]->getChromosome(range(GlobalCppRandomEngine::engine));
				
				if (selectChromosome1->getFitness(this->target) > selectChromosome2->getFitness(this->target)) 
				{
					this->selectedChromosome[i][j] = selectChromosome1;
				} 
				else 
				{
					this->selectedChromosome[i][j] = selectChromosome2;
				}
			}
		}	
    }

    void MainProcess::crossover() 
	{
		for(int i=0; i<this->numberOfPopulation; i++)
		{
			for (unsigned long j = 0; j < this->kill; j++) 
			{
				this->newChromosome[i][j] = this->selectedChromosome[i][2 * j]->crossover(this->selectedChromosome[i][1 + 2 * j], this->limit);
			}
		}
    }

    void MainProcess::mutation() 
	{
        if (0 >= this->r) 
		{
            return;
        }
		
		cout << " " << this->pm << " ";
        
		for(int i=0; i<this->numberOfPopulation; i++)
		{
			for (unsigned long j = 0; j < this->kill; j++) 
			{
				this->newChromosome[i][j]->mutation(this->pm, this->limit);
			}
		}
    }

    void MainProcess::generated() 
	{
        if (1 != this->keep) 
		{
			for(int i=0; i<this->numberOfPopulation; i++)
			{
				for (unsigned long j = this->keep; j < this->numberOfChromosome; j++) 
				{
					this->populations[i]->replaceChromosome(j, this->newChromosome[i][j - this->keep]);
				}
			}
        } 
		else 
		{
			for(int i=0; i<this->numberOfPopulation; i++)
			{
				unsigned long replaceOffset = 0, newChromosomePoolOffset = 0;
				Chromosome* maxFitnessChromosome = this->populations[i]->getMaxFitnessChromosome();
				
				for (unsigned long j = 0; j < this->numberOfChromosome; j++) 
				{
					if ((void*)(this->populations[i]->getChromosome(replaceOffset)) != (void*)maxFitnessChromosome) 
					{
						this->populations[i]->replaceChromosome(replaceOffset, this->newChromosome[i][newChromosomePoolOffset]);
						newChromosomePoolOffset++;
					}
					
					replaceOffset++;
				}

				replaceOffset = 0; newChromosomePoolOffset = 0;
			}
        }
    }

	TransferMatrix MainProcess::getTarget()
	{
		return this->target;
	}

    void MainProcess::freeMemory() 
	{
		if(nullptr != this->populations)
		{
			for(int i=0; i<this->numberOfPopulation; i++)
			{
				if (nullptr != this->populations[i]) 
				{
					delete this->populations[i];
					this->populations[i] = nullptr;
				}
				
				// selectedChromosome 指向 Population 实例里面的染色体，Population 会删除不需要在这里单独删染色体
				if (nullptr != this->selectedChromosome[i]) 
				{
					delete[] this->selectedChromosome[i];
					this->selectedChromosome[i] = nullptr;
				}
				
				// 这些染色体会替换 Population 里面的，Population 会在替换时自己删掉，不需要在这里单独删染色体
				if (nullptr != this->newChromosome[i]) 
				{
					delete[] this->newChromosome[i];
					this->newChromosome[i] = nullptr;
				}
			}

			delete[] this->newChromosome;
			delete[] this->selectedChromosome;
			delete[] this->populations;
			this->newChromosome = nullptr;
			this->selectedChromosome = nullptr;
			this->populations = nullptr;
		}
    }
}
