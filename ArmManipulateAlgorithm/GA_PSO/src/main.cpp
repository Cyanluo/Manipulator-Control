#include <iostream>
#include "GeneticAlgorithm/MainProcess.h"
#include <iomanip>
#include <vector>

#include <random>
#include "GeneticAlgorithm/Utils/GlobalCppRandomEngine.h"

using namespace GeneticAlgorithm;
using namespace std;

int main()
{
	cout  << fixed << setprecision(2);

	Matrix<float, JOINTN, 4> dh;
	dh << -PI/2,   0,   2,   0,
			PI/2,   0,  0, PI/2,
			0,     10,   0,  0,
			0,     20,  0,   0,
			0,     30,   0,  0;

	TransferMatrix wf;

	DH_MechanicalArm<5, 5> arm(dh, wf);

	VectorXf runParams(JOINTN, 1);
	runParams << RADIAN(-60),RADIAN(50),RADIAN(-10),RADIAN(-23),RADIAN(0);

	TransferMatrix target = arm.forward(runParams);

    MainProcess mainProcess = MainProcess();
	MatrixXd limit(5, 2);

	limit << -90, 90,
			-90, 90,
			-90, 90,
			-90, 10,
			0, 0;

	mainProcess.setDebug(true);
    mainProcess.run(
        150, // 种群大小
        5, // 染色体长度
		limit, // 每个基因的限制
        80, // 最大迭代次数
        10, // 停止迭代适应度
        40, // 每次迭代保留多少个上一代的高适应度个体
        0.8, // 变异概率，随便变动范围系数
		target // 寻优的目标
    );

	cout << "target:" << endl << target << endl;

    return 0;
}

/*
 mainProcess.run(
        120, // 种群大小
        5, // 染色体长度
        -180, // 初始范围
        180, // 初始范围
        1000, // 最大迭代次数
        50, // 停止迭代适应度
        25, // 每次迭代保留多少个上一代的高适应度个体
        0.8 // 变异概率，随便变动范围系数
    );
*/

/* 多线程版本

int main()
{
    Multithreading mainProcess = Multithreading(4);
    mainProcess.setDebug(false);
    mainProcess.run(
        50, // 种群大小
        2, // 染色体长度
        -2.048, // 初始范围
        2.048, // 初始范围
        0, // 最大迭代次数
        99.9999, // 停止迭代适应度
        5, // 每次迭代保留多少个上一代的高适应度个体
        0.015 // 变异概率，随便变动范围系数
    );
    cout << "Max fitness=" << mainProcess.getMaxFitness() << endl;
    for (int i = 0; i < 100; i++) {
        //mainProcess.exchange();
        mainProcess.runContinue(50, 99.9999, 5, 0.015);
        cout << i << ",Max fitness=" << mainProcess.getMaxFitness() << endl;
    }
    return 0;
}

*/
