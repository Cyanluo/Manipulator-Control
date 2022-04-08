#include <iostream>
#include "GeneticAlgorithm/MainProcess.h"
#include <iomanip>
#include <vector>

#include <random>
#include "GeneticAlgorithm/Utils/GlobalCppRandomEngine.h"
#include "GeneticAlgorithm/Multithreading.h"

#include <unistd.h>
#include <chrono>

using namespace GeneticAlgorithm;
using namespace std;

int main()
{
    cout  << fixed << setprecision(5);

	Matrix<float, JOINTN, 4> dh;
	dh << -PI/2,   0,   90,   0,
			PI/2,   0,  0,   PI/2,
			0,     105,   0,  0,
			0,     98,  0,   0,
			0,     150,   0,  0;

	TransferMatrix wf;

	DH_MechanicalArm<JOINTN, JOINTN> arm(dh, wf);

	VectorXf runParams(JOINTN, 1);
	runParams << RADIAN(-20),RADIAN(-30),RADIAN(50),RADIAN(-10),RADIAN(0);
	TransferMatrix target = arm.forward(runParams);
	cout << runParams << endl;
    MainProcess mainProcess = MainProcess();
	MatrixXd limit(JOINTN+1, 2);

	limit << -90, 90,
			-90, 90,
			-90, 90,
			-90, 10,
			-90, 90,
			-9, 9; // 粒子群速度限制

	mainProcess.setDebug(true);
    // mainProcess.setPSO(
	// 	0.2, // w 惯性权重
	// 	1.8, // c1 种群内权重
	// 	2	 // c2 种群间权重
	// );
	
	auto start0 = chrono::steady_clock::now();
	
	// mainProcess.run(
	// 	5,	// 种群数量
    //     25, // 种群大小
    //     JOINTN, // 染色体长度
	// 	limit, // 每个基因的限制
    // 	40, // 最大迭代次数
    //     40, // 停止迭代适应度
    //     22, // 每次迭代保留多少个上一代的高适应度个体
    //     1.1, // 变异调整参数
	// 	target // 寻优的目标
    // );

	mainProcess.run(
		1,	// 种群数量
        125, // 种群大小
        JOINTN, // 染色体长度
		limit, // 每个基因的限制
        100, // 最大迭代次数
        99.9, // 停止迭代适应度
        25, // 每次迭代保留多少个上一代的高适应度个体
        1.1, // 变异调整参数
		target // 寻优的目标
    );

	cout << "target:" << endl << target << endl;
	
	auto end0 = chrono::steady_clock::now();
	chrono::duration<double> elapsed0 = end0 - start0;
	int time =  elapsed0.count()*1000;
	cout << "Timeeeeeeeeeeeeeeeeeeeeeeeee=============================solve func time: " << time << "ms" << endl;
	
	 //cout << mainProcess.getPlotData() << endl;

	return 0;
}

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
