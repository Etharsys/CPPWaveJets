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
    cam.showWidget("dots", viz::WCloud(_dots, viz::Color::black()));
}

Point3d CloudDots::centered_p()
{
    const auto compare_by_dist = 
        [](const Point3d& p1, const Point3d& p2) 
        {
            return abs(p1.x) + abs(p1.y) < abs(p2.x) + abs(p2.y); 
        };

    return *std::min_element(_dots.begin(), _dots.end(), compare_by_dist);
}

Point3d CloudDots::random_origin()
{
    /*double x = rand() % ((int) MAX_CLOUD_DOTS_X - (int) MIN_CLOUD_DOTS_X + 1) - 10;
    double y = rand() % ((int) MAX_CLOUD_DOTS_Y - (int) MIN_CLOUD_DOTS_Y + 1) - 10;
    return Point3d { x, y, 0. };*/
    return create_random_point_on_plan(); // pas utile ?
}

Vec2d CloudDots::random_2d_vector()
{
    double a = rand() % 201 - 100; 
    double b = rand() % 201 - 100; 
    return Vec2d { a, b };
}

Point3d CloudDots::create_random_point_on_plan()
{
    double x = rand() % ((int) MAX_CLOUD_DOTS_X - (int) MIN_CLOUD_DOTS_X + 1) - 10;
    double y = rand() % ((int) MAX_CLOUD_DOTS_Y - (int) MIN_CLOUD_DOTS_Y + 1) - 10;
    return Point3d { x, y, 0. };
}

void CloudDots::create_all_random_points_on_plan()
{
    for (unsigned int i = 0; i < MAX_CLOUD_POINTS; ++i)
    {
        _dots.at(i) = (create_random_point_on_plan());
    }
}

void CloudDots::generate_random_noise()
{
    transform(_dots.begin(), _dots.end(), _dots.begin(), 
        [](const Point3d& p) 
        {
            double z = rand() % ((int) MAX_CLOUD_DOTS_X - (int) MIN_CLOUD_DOTS_X + 1) - 10;
            return Point3d { p.x, p.y, z * MAX_CLOUD_DOTS_NOISE}; 
        }
    );
}