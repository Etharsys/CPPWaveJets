#include "CloudDots.hpp"

#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;
using namespace viz;

void CloudDots::generate_random_cloud()
{
    create_all_random_points_on_plan();
    generate_random_noise();
}

void CloudDots::display(Viz3d& cam)
{
    /*for (const auto& p : _dots)
    {
        cout << p << endl;
    }*/
    //cout << "ori : " << _origin << endl;
    //cout << "vecA, B : " << _vecA << ", " << _vecB << endl;
    cam.showWidget("dots", WCloud(_dots, Color::black()));
    //Point3d p1 { _vecA[0] * 10, _vecA[1] * 10, _vecA[2] * 10 };
    //cam.showWidget("v1", WLine(_origin, p1 + _origin, Color::blue()));
    //Point3d p2 { _vecB[0] * 10, _vecB[1] * 10, _vecB[2] * 10 };
    //cam.showWidget("v2", WLine(_origin, p2 + _origin, Color::green()));
}

Point3d CloudDots::centered_p()
{
    const auto compare_by_dist = 
        [this](const Point3d& p1, const Point3d& p2) 
        {
            return pow(p1.x - _origin.x, 2) + pow(p1.y - _origin.y, 2) 
                 < pow(p2.x - _origin.x, 2) + pow(p2.y - _origin.y, 2); 
        };

    return *std::min_element(_dots.begin(), _dots.end(), compare_by_dist);
}

Point3d CloudDots::random_origin()
{
    double x = rand() % (2 * MAX_DOTS_CLOUD_RADIUS + 1) - MAX_DOTS_CLOUD_RADIUS;
    double y = rand() % (2 * MAX_DOTS_CLOUD_RADIUS + 1) - MAX_DOTS_CLOUD_RADIUS;
    double z = rand() % (2 * MAX_DOTS_CLOUD_RADIUS + 1) - MAX_DOTS_CLOUD_RADIUS;
    return Point3d { x, y, z };
}

Vec3d CloudDots::random_3d_vector()
{
    double a = rand() % (2 * MAX_DOTS_CLOUD_RADIUS + 1) - MAX_DOTS_CLOUD_RADIUS;
    double b = rand() % (2 * MAX_DOTS_CLOUD_RADIUS + 1) - MAX_DOTS_CLOUD_RADIUS;
    double c = rand() % (2 * MAX_DOTS_CLOUD_RADIUS + 1) - MAX_DOTS_CLOUD_RADIUS; 
    Vec3d v { a, b, c };
    return v / sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[3], 2));
}

void CloudDots::create_all_random_points_on_plan()
{
    for (unsigned int i = 0; i < MAX_CLOUD_POINTS; ++i)
    {
        _dots.at(i) = create_random_point_on_plan();
    }
}

Point3d CloudDots::create_random_point_on_plan()
{
    auto alpha = (rand() % (2 * MAX_DOTS_CLOUD_RADIUS + 1) - MAX_DOTS_CLOUD_RADIUS);
    auto beta  = (rand() % (2 * MAX_DOTS_CLOUD_RADIUS + 1) - MAX_DOTS_CLOUD_RADIUS);
    double x = (alpha * _vecA[0] + beta * _vecB[0] + _origin.x);
    double y = (alpha * _vecA[1] + beta * _vecB[1] + _origin.y);
    double z = (alpha * _vecA[2] + beta * _vecB[2] + _origin.z);
    return Point3d { x, y, z };
}

void CloudDots::generate_random_noise()
{
    transform(_dots.begin(), _dots.end(), _dots.begin(), 
        [](const Point3d& p) 
        {
            double z = rand() % (2 * MAX_DOTS_CLOUD_RADIUS + 1) - MAX_DOTS_CLOUD_RADIUS;
            return Point3d { p.x, p.y, p.z + z * MAX_CLOUD_DOTS_NOISE}; 
        }
    );
}