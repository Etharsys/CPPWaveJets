#include "CylinderCloud.hpp"


using namespace std;
using namespace cv;
using namespace viz;


void CylinderCloud::generate_random_cloud()
{

    create_all_random_points_on_plan();
    generate_transformation_mat();
    compute_transformation_matrix();
    //generate_random_noise();
}

void CylinderCloud::generate_transformation_mat()
{
    double alpha = generateUniformDouble(0, 2 * M_PI);
    double beta  = generateUniformDouble(0, 2 * M_PI);
    double gamma = generateUniformDouble(0, 2 * M_PI);

    _T = Eigen::Translation3d(_origin.x, _origin.y, _origin.z) 
        * Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(beta , Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ());
}

void CylinderCloud::compute_transformation_matrix()
{
    for (auto& dot : _dots)
    {
        Eigen::Vector3d eigen_dot { dot.x, dot.y, dot.z };
        eigen_dot = _T * eigen_dot;
        dot = Point3d { eigen_dot(0), eigen_dot(1), eigen_dot(2) };
    }
}

void CylinderCloud::display(Viz3d& cam)
{ //opencv display part
    cam.showWidget("dots", WCloud(_dots, Color::black()));
    //cam.showWidget("orig", WCloud(vector {_origin}, Color::green()));
}

void CylinderCloud::display(cv::viz::Viz3d& cam, const cv::Point3d& p, int radius)
{
    auto dots = dots_to_vector(p, radius);
    cam.showWidget("dots_in_radius", WCloud(dots, Color::blue()));
    cam.showWidget("selected_p", WCloud(vector {p}, Color::red()));
}

Point3d CylinderCloud::centered_p()
{
    const auto compare_by_dist = 
        [this](const Point3d& p1, const Point3d& p2) 
        {
            return norm(p1 - _origin) < norm(p2 - _origin);
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
    return Point3d { radius * cos(theta), radius * sin(theta), z };
}

Point3d CylinderCloud::create_random_point_on_plan(double z)
{
    auto radius = generateUniformDouble(0, MAX_DOTS_CLOUD_RADIUS);
    auto theta  = generateUniformDouble(0, 2*M_PI);

    return Point3d { radius * cos(theta), radius * sin(theta), z };
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

vector<Point3d> CylinderCloud::dots_to_vector()
{
    vector<Point3d> dots_vec;

    for (const auto& dot : _dots)
    {
        dots_vec.emplace_back(dot);
    }

    return dots_vec;
}

std::vector<cv::Point3d> CylinderCloud::dots_to_vector(const cv::Point3d& p, int radius)
{
    vector<cv::Point3d> dots;
    std::copy_if(_dots.begin(), _dots.end(), std::back_inserter(dots), 
        [p, radius](const Point3d& p1) {
        return norm(p - p1) <= radius;
    });
    return dots;
}
