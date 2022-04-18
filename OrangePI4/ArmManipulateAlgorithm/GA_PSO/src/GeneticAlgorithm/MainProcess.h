#ifndef GENETICALGORITHM_MAINPROCESS_H
#define GENETICALGORITHM_MAINPROCESS_H

#include "Population.h"
#include "Chromosome.h"
#include <string>

namespace GeneticAlgorithm {

    class MainProcess {

    public:
        // 构造方法
        MainProcess();
        // 销毁对象时用于释放内存
        ~MainProcess();

        // 主流程运行
        void run(
			unsigned long numberOfPopulation,
            unsigned long numberOfChromosome, // 种群中个体数量
            unsigned long lengthOfChromosome, // 每个个体的基因长度
            MatrixXd& limit,
            unsigned long maxLoop, // 最大迭代次数
            long double stopFitness, // 达到多大的适应度就立刻停止迭代
            unsigned long keep, // 每次迭代保留多少个上一代的个体
            long double r, // 基因突变的概率
			TransferMatrix& target
        );

        // 设置debug模式，为true的时候打印调试信息
        void setDebug(bool enableDebug);
        // 获取迭代次数。如果在一开始初始化的那代种群就达到停止的条件，那么返回0
        unsigned long getLoopNumber();
        // 获取最大的适应度
        long double getMaxFitness();
		long double getMaxFitness(int& indexPopulation);
        // 获取Fitness最大值的Chromosome
        Chromosome* getMaxFitnessChromosome();
        // 替换一个不是最好的个体为指定的个体
        void replaceChromosome(Chromosome* chromosome);
		TransferMatrix getTarget();
		void setPSO(double w, double c1, double c2);
        string getPlotData();
        VectorXf getResultRunParams();

        void disablePSO();
    private:
		unsigned long numberOfPopulation;
        unsigned long numberOfChromosome;
        unsigned long lengthOfChromosome;

		MatrixXd limit;
        unsigned long keep;
        unsigned long kill;
        unsigned long loopNow;
        long double maxFitness;
		double minInaccuracy;
        VectorXf resultRunParams;
        Chromosome*** selectedChromosome = nullptr;
        Chromosome*** newChromosome = nullptr;
        long double r;
		long double pm;
        Population** populations = nullptr;
        bool debug = false;
		TransferMatrix target;
        void init(unsigned long numberOfPopulation);
        void sort();
        void select();
        void crossover();
        void mutation();
        void generated();
		void PSO();
        void freeMemory();

        string plotData;

		bool isPSOEabled;
		double w;
		double c1;
		double c2;
    };

}

#endif
