#ifndef GENETICALGORITHM_CHROMOSOME_H
#define GENETICALGORITHM_CHROMOSOME_H

#include "DH_MechanicalArm.h"
#include<math.h>

namespace GeneticAlgorithm {

const double PI = 3.1415926;
#define RADIAN(a) (PI/180*a) 
const int JOINTN = 5;

    /* 染色体
     */
    class Chromosome {

    public:

        /**
         * 创建染色体，其中实数初始化为不确定的值
         *
         * 入参是染色体中的实数个数，一个染色体保存多个实数
         *
         * @param unsigned long numberOfReal
         */
        Chromosome(unsigned long numberOfReal);

        /**
         * 删除染色体，释放内存
         */
        ~Chromosome();

        /**
         * 设置给定位置的实数大小
         *
         * @param unsigned long offset 位置，大于等于0小于染色体的长度numberOfReal
         * @param long double value 设置的实数的值
         * @return bool 成功返回 true
         */
        bool setGene(unsigned long offset, long double value);
        bool setVelocity(unsigned long offset, long double value);

        /**
         * 获取给定位置的实数大小
         *
         * @param unsigned long offset 位置，大于等于0小于染色体的长度numberOfReal
         * @return long double value 实数的值
         */
        long double getGene(unsigned long offset);
        long double getVelocity(unsigned long offset);

        /**
         * 打印调试信息
         *
         * @return void
         */
        void dump();

        /**
         * 获取长度，也就是染色体内保存的实数个数
         *
         * @return unsigned long
         */
        unsigned long getLength();

        /**
         * 获取适应度
         *
         * @return long double
         */
        long double getFitness(TransferMatrix& target);

        /**
         * 与另一个染色体交叉，返回新的染色体
         *
         * @param Chromosome* another 另一个染色体对象
         * @return Chromosome* 新的染色体对象，需要手动释放内存
         */
        Chromosome* crossover(Chromosome* another, MatrixXd& limit);

        /**
         * 以一定的概率r变异
         *
         * @param long double r
         * @return void
         */
        void mutation(long double r, MatrixXd& limit);
		void limitV(long double min, long double max);
		void PSO(long double*& pbest,
		 		 long double*& gbest,
				 long double w,
				 long double c1,
				 long double c2,
				 MatrixXd& limit);
		
		void getData(long double*& dst);
		double getInaccuracy();
    private:
		void calcInaccuracy(TransferMatrix& l,
						 TransferMatrix& target,
						 double& postionInaccuracy,
						 double& postureInaccuracy);
		void limiting(long double*& data, MatrixXd& limit);
		void limiting(long double*& data, long double min, long double max);
		void arrayMUL(long double*& dst, long double*& l, long double c);
		void arraySUB(long double*& dst, long double*& l, long double*& r);
        void arrayADD(long double*& dst, long double*& l, long double*& r);
		void arrayCOPY(long double*& src, long double*& dst);
		// @var unsigned long 保存了此染色体的长度 
        unsigned long lengthOfData;

        // @var long double* 保存了此染色体中基因的信息 
        long double* dataArray;

		// @var long double* 保存了PSO算法的速度
		long double* velocity;

        // @var bool 为true时表示计算Fitness后缓存了计算结果，可以不用重复算
        bool isFitnessCached = false;

        // @var long double 缓存的上一次的适应度计算结果。需要判断isFitnessCached以确定确实缓存下来了。
        long double fitnessCached;
		double inaccuracy;

		JMechArm<5, 5>* arm;
		TransferMatrix target;
    };

}

#endif
