#pragma once

// max noise percentage for cloud dots
constexpr double MAX_CLOUD_DOTS_NOISE = 5. / 100;

// max size of cloud points plan
constexpr double MAX_CLOUD_DOTS_X = +10.;
constexpr double MAX_CLOUD_DOTS_Y = +10.;
constexpr double MIN_CLOUD_DOTS_X = -10.;
constexpr double MIN_CLOUD_DOTS_Y = -10.;

// max points for cloud points
constexpr unsigned int MAX_CLOUD_POINTS = 200;

// wavejet order demo
constexpr unsigned int ORDER = 2;