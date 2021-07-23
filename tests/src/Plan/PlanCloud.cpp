#include "PlanCloud.hpp"


using namespace std;
using namespace cv;
using namespace viz;


void CloudPoints::generate_random_cloud()
{
    make_vector_square();
    create_all_random_points_on_plan();
    generate_random_noise();
}

void CloudPoints::display(Viz3d& cam)
{
    // out stream
    
    /*for (const auto& p : _points)
    {
        cout << p << endl;
    }
    //cout << "ori : " << _origin << endl;
    //cout << "vecA : " << _vecA  << endl;
    //cout << "vecB : " << _vecB << endl;
    //cout << "colA : " << _colA << endl;*/
    
    
    // opencv
    cam.showWidget("plan points", WCloud(_points, Color::black()));
    /*
    Point3d p1 { _vecA[0] * 10, _vecA[1] * 10, _vecA[2] * 10 };
    cam.showWidget("v1", WLine(_origin, p1 + _origin, Color::green()));
    Point3d p2 { _vecB[0] * 10, _vecB[1] * 10, _vecB[2] * 10 };
    cam.showWidget("v2", WLine(_origin, p2 + _origin, Color::orange()));
    Point3d p3 { _colA[0] * 10, _colA[1] * 10, _colA[2] * 10 };
    cam.showWidget("v3", WLine(_origin, p3 + _origin, Color::blue()));
    */
}

Point3d CloudPoints::centered_p()
{
    const auto compare_by_dist = 
        [this](const Point3d& p1, const Point3d& p2) 
        {
            return (pow(p1.x - _origin.x, 2) 
                  + pow(p1.y - _origin.y, 2) 
                  + pow(p1.z - _origin.z, 2))
                 < (pow(p2.x - _origin.x, 2) 
                  + pow(p2.y - _origin.y, 2) 
                  + pow(p2.z - _origin.z, 2));
        };

    return *std::min_element(_points.begin(), _points.end(), compare_by_dist);
}

void CloudPoints::make_vector_square()
{
    Eigen::Vector3d a { _vecA[0], _vecA[1], _vecA[2] };
    Eigen::Vector3d b { _vecB[0], _vecB[1], _vecB[2] };
    auto r = a.cross(b).cross(a);
    _vecB = cv::Point3d { r(0), r(1), r(2) };
    normalize(_vecA, _vecA);
    normalize(_vecB, _vecB);
}

Point3d CloudPoints::random_origin()
{
    return Point3d { generateUniformDouble(-MAX_POINTS_CLOUD_RADIUS, MAX_POINTS_CLOUD_RADIUS), 
                     generateUniformDouble(-MAX_POINTS_CLOUD_RADIUS, MAX_POINTS_CLOUD_RADIUS),
                     generateUniformDouble(-MAX_POINTS_CLOUD_RADIUS, MAX_POINTS_CLOUD_RADIUS) };
}

Vec3d CloudPoints::random_3d_vector()
{
    return Vec3d { generateUniformDouble(-MAX_POINTS_CLOUD_RADIUS, MAX_POINTS_CLOUD_RADIUS), 
                   generateUniformDouble(-MAX_POINTS_CLOUD_RADIUS, MAX_POINTS_CLOUD_RADIUS), 
                   generateUniformDouble(-MAX_POINTS_CLOUD_RADIUS, MAX_POINTS_CLOUD_RADIUS) };
}

void CloudPoints::create_all_random_points_on_plan()
{
    for (unsigned int i = 0; i < MAX_CLOUD_POINTS; ++i)
    {
        _points.at(i) = create_random_point_on_plan();
    }
}

Point3d CloudPoints::create_random_point_on_plan()
{
    auto alpha = generateUniformDouble(-MAX_POINTS_CLOUD_RADIUS, MAX_POINTS_CLOUD_RADIUS);
    auto beta  = generateUniformDouble(-MAX_POINTS_CLOUD_RADIUS, MAX_POINTS_CLOUD_RADIUS);
    double x = (alpha * _vecA[0] + beta * _vecB[0] + _origin.x);
    double y = (alpha * _vecA[1] + beta * _vecB[1] + _origin.y);
    double z = (alpha * _vecA[2] + beta * _vecB[2] + _origin.z);
    return Point3d { x, y, z };
}

void CloudPoints::generate_random_noise()
{
    transform(_points.begin(), _points.end(), _points.begin(), 
        [](const Point3d& p) 
        {
            double z = generateUniformDouble(-MAX_POINTS_CLOUD_RADIUS, MAX_POINTS_CLOUD_RADIUS) 
                        * MAX_CLOUD_POINTS_NOISE;
            return Point3d { p.x, p.y, p.z + z}; 
        }
    );
}

vector<Point3d> CloudPoints::points_to_vector()
{
    vector<Point3d> points_vec;

    for (const auto& point : _points)
    {
        points_vec.emplace_back(point);
    }

    return points_vec;
}