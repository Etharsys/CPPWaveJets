#pragma once

#include <random>
#include <chrono>

// max noise percentage for cloud dots
constexpr double MAX_CLOUD_DOTS_NOISE = 20. / 100;

// max size of cloud points plan
constexpr int MAX_DOTS_CLOUD_RADIUS = 10;

// max points for cloud points
constexpr unsigned int MAX_CLOUD_POINTS = 1000;

// wavejet order demo
constexpr unsigned int ORDER = 2;

// select seed from time
inline double generateUniformDouble(){
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

    // select a random generator engine
    std::default_random_engine generator(seed);

    // uniform real distribution (ici entre 1 et 6, sur des float)
    std::uniform_real_distribution<double> uniformRealDistribution(
        -MAX_DOTS_CLOUD_RADIUS, MAX_DOTS_CLOUD_RADIUS);
    return uniformRealDistribution(generator);
}