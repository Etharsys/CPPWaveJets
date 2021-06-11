#pragma once

#include <random>
#include <chrono>


// max size of cloud points plan
constexpr int MAX_DOTS_CLOUD_RADIUS = 10;


// select seed from time
inline double generateUniformDouble(double min, double max) 
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

    // select a random generator engine
    std::default_random_engine generator(seed);

    // uniform real distribution (ici entre 1 et 6, sur des float)
    std::uniform_real_distribution<double> uniformRealDistribution(min, max);
    return uniformRealDistribution(generator);
}

inline double generateUniformDouble() 
{
    return generateUniformDouble(-MAX_DOTS_CLOUD_RADIUS, MAX_DOTS_CLOUD_RADIUS);
}

inline double generateUniformDouble(double max) 
{
    return generateUniformDouble(0, max);
}
