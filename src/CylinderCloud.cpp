#include "CylinderCloud.hpp"


using namespace std;
using namespace cv;
using namespace viz;


void CylinderCloud::generate_random_cloud()
{
    create_all_random_points_on_plan();
    //generate_random_noise();
}

void CylinderCloud::display(Viz3d& cam)
{
    // opencv
    cam.showWidget("dots", WCloud(_dots, Color::black()));
    cam.showWidget("orig", WCloud(vector {_origin}, Color::green()));
    cam.showWidget("rand", WCloud(vector {centered_p()}, Color::red()));
}

Point3d CylinderCloud::centered_p()
{
    const auto compare_by_dist = 
        [this](const Point3d& p1, const Point3d& p2) 
        {
            return abs(abs(p1.z) - abs(_origin.z)) < 
                   abs(abs(p2.z) - abs(_origin.z));
        };

    return *std::min_element(_dots.begin(), _dots.end(), compare_by_dist);
}

Point3d CylinderCloud::random_origin()
{
    return Point3d { generateUniformDouble(), 
                     generateUniformDouble(),
                     generateUniformDouble() };
}

void CylinderCloud::create_all_random_points_on_plan()
{
    auto step = MAX_CYLINDER_CLOUD / MAX_CLOUD_POINTS;
    for (u_int i = 0; i < MAX_CYLINDER_CLOUD; i+=step)
    {
        _dots.at(i  ) = create_random_point_on_plan( _height);
        _dots.at(i+1) = create_random_point_on_plan(-_height);
        
        for (u_int j = i + 2; j < i + step; ++j)
        {
            _dots.at(j) = create_random_point_on_cyl();
        }
    }
}

cv::Point3d CylinderCloud::create_random_point_on_cyl()
{
    auto theta  = generateUniformDouble(0, 2*M_PI);
    auto radius = MAX_DOTS_CLOUD_RADIUS;
    auto z      = generateUniformDouble(-_height, _height);
    return Point3d { radius * cos(theta) + _origin.x, 
                     radius * sin(theta) + _origin.y, 
                     z + _origin.z };
}

Point3d CylinderCloud::create_random_point_on_plan(double z)
{
    auto radius = generateUniformDouble(0, MAX_DOTS_CLOUD_RADIUS);
    auto theta  = generateUniformDouble(0, 2*M_PI);
    return Point3d { radius * cos(theta) + _origin.x, 
                     radius * sin(theta) + _origin.y, 
                     z + _origin.z };
}

void CylinderCloud::generate_random_noise()
{
    transform(_dots.begin(), _dots.end(), _dots.begin(), 
        [](const Point3d& p) 
        {
            double z = generateUniformDouble() * MAX_CLOUD_DOTS_NOISE;
            return Point3d { p.x, p.y, p.z + z}; 
        }
    );
}