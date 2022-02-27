#include <iostream>
#include "GeneticAlgorithm/MainProcess.h"
#include <iomanip>
#include <vector>

#include <random>
#include "GeneticAlgorithm/Utils/GlobalCppRandomEngine.h"
#include "GeneticAlgorithm/Multithreading.h"

#include <unistd.h>

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
		3,	// 种群数量
        150, // 种群大小
        5, // 染色体长度
		limit, // 每个基因的限制
        50, // 最大迭代次数
        10, // 停止迭代适应度
        40, // 每次迭代保留多少个上一代的高适应度个体
        1, // 变异调整参数
		target // 寻优的目标
    );

	cout << "target:" << endl << target << endl;

    return 0;
}

// 多线程版本

// int main()
// {
// 	cout  << fixed << setprecision(2);

// 	Matrix<float, JOINTN, 4> dh;
// 	dh << -PI/2,   0,   2,   0,
// 			PI/2,   0,  0, PI/2,
// 			0,     10,   0,  0,
// 			0,     20,  0,   0,
// 			0,     30,   0,  0;

// 	TransferMatrix wf;

// 	DH_MechanicalArm<5, 5> arm(dh, wf);

// 	VectorXf runParams(JOINTN, 1);
// 	runParams << RADIAN(-60),RADIAN(50),RADIAN(-10),RADIAN(-23),RADIAN(0);

// 	TransferMatrix target = arm.forward(runParams);

// 	MatrixXd limit(5, 2);

// 	limit << -90, 90,
// 			-90, 90,
// 			-90, 90,
// 			-90, 10,
// 			0, 0;

//     Multithreading mainProcess = Multithreading(3);
//     mainProcess.setDebug(true);
//     mainProcess.run(
//         150, // 种群大小
//         5, // 染色体长度
// 		limit, // 每个基因的限制
//         50, // 最大迭代次数
//         30, // 停止迭代适应度
//         40, // 每次迭代保留多少个上一代的高适应度个体
//         1, // 变异概率，随便变动范围系数
// 		target // 寻优的目标
//     );
    
// 	cout << "Max fitness=" << mainProcess.getMaxFitness() << endl;
    
// 	cout << "target:" << endl << target << endl;
	
// 	for (int i = 0; i < 100; i++) 
// 	{
//         //mainProcess.exchange();
//         mainProcess.runContinue(150, 30, 40, 1);
//         cout << i << ",Max fitness=" << mainProcess.getMaxFitness() << endl;
//     }

//     return 0;
// }
