#include "GlobalCppRandomEngine.h"
#include <random>
#include <iostream>

namespace GeneticAlgorithm::Utils {

    std::default_random_engine GlobalCppRandomEngine::engine;
    //std::default_random_engine GlobalCppRandomEngine::engine(time(nullptr));

}
