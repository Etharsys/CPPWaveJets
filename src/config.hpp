#pragma once

#include <random>
#include <chrono>

// max noise percentage for cloud dots
constexpr double MAX_CLOUD_DOTS_NOISE = 20. / 100;

// max size of cloud points plan
constexpr int MAX_DOTS_CLOUD_RADIUS = 10;

// max points for cloud points
constexpr unsigned int MAX_CLOUD_POINTS   = 200;
constexpr unsigned int MAX_CYLINDER_CLOUD = 
    MAX_CLOUD_POINTS * 2 + // 2 faces
    MAX_CLOUD_POINTS * 4;  // cyl

// wavejet order demo
constexpr unsigned int ORDER = 2;

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

constexpr unsigned int ANGLE_DIVISION  = 64;
constexpr unsigned int RADIUS_DIVISION = 12;
constexpr unsigned int WAVEJET_DISPLAY_RADIUS = 3;