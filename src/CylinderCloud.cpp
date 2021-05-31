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
    cv::Point3d vec { _vec(0) * 10, _vec(1) * 10, _vec(2) * 10};
    cv::Point3d ori { 0, 0, 0 };
    cam.showWidget("t2", cv::viz::WLine(ori, vec, cv::viz::Color::blue()));
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

Vec3d CylinderCloud::random_3d_vector()
{
    Vec3d vec { generateUniformDouble(), 
                   generateUniformDouble(), 
                   generateUniformDouble() };
    normalize(vec, vec);
    return vec;
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
    apply_rotation();
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

void CylinderCloud::apply_rotation()
{
    double pitch =  _vec.dot(Vec3d { 1, 0, 0 });
    double roll  =  _vec.dot(Vec3d { 0, 1, 0 });
    double yaw   =  _vec.dot(Vec3d { 0, 0, 1 });

    auto cosa = cos(yaw);
    auto sina = sin(yaw);

    auto cosb = cos(pitch);
    auto sinb = sin(pitch);

    auto cosc = cos(roll);
    auto sinc = sin(roll);

    auto Axx = cosa*cosb;
    auto Axy = cosa*sinb*sinc - sina*cosc;
    auto Axz = cosa*sinb*cosc + sina*sinc;

    auto Ayx = sina*cosb;
    auto Ayy = sina*sinb*sinc + cosa*cosc;
    auto Ayz = sina*sinb*cosc - cosa*sinc;

    auto Azx = -sinb;
    auto Azy = cosb*sinc;
    auto Azz = cosb*cosc;

    for (auto& p : _dots)
    {
        auto px = p.x;
        auto py = p.y;
        auto pz = p.z;

        p.x = Axx*px + Axy*py + Axz*pz;
        p.y = Ayx*px + Ayy*py + Ayz*pz;
        p.z = Azx*px + Azy*py + Azz*pz;
    }
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
