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
    dots_to_vector();
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
    for (auto& dot : _face1)
        dot = compute_transformation_matrix(dot);
    for (auto& dot : _face2)
        dot = compute_transformation_matrix(dot);
    for (auto& dot : _tube)
        dot = compute_transformation_matrix(dot);
}

Point3d CylinderCloud::compute_transformation_matrix(const Point3d& dot)
{
    Eigen::Vector3d eigen_dot { dot.x, dot.y, dot.z };
    eigen_dot = _T * eigen_dot;
    return Point3d { eigen_dot(0), eigen_dot(1), eigen_dot(2) };
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

    return *std::min_element(_tube.begin(), _tube.end(), compare_by_dist);
}

Point3d CylinderCloud::random_origin()
{
    return Point3d { generateUniformDouble(), 
                     generateUniformDouble(),
                     generateUniformDouble() };
}

void CylinderCloud::create_all_random_points_on_plan()
{
    for (u_int i = 0; i < DOTS_THRESHOLD * 2; ++i)
    {
        _face1.at(i) = create_random_point_on_plan( _height);
        _face2.at(i) = create_random_point_on_plan(-_height);
        
        _tube.at(2 * i)     = create_random_point_on_cyl();
        _tube.at(2 * i + 1) = create_random_point_on_cyl();
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

void CylinderCloud::dots_to_vector()
{
    /*for_each(_face1.begin(),_face1.end(), std::back_inserter(_dots));
    for_each(_face2.begin(),_face2.end(), std::back_inserter(_dots));
    for_each( _tube.begin(), _tube.end(), std::back_inserter(_dots));*/
    //for (const auto& dot : _face1) _dots.emplace_back(dot);
    //for (const auto& dot : _face2) _dots.emplace_back(dot);
    for (const auto& dot :  _tube) _dots.emplace_back(dot);
}

std::vector<cv::Point3d> CylinderCloud::dots_to_vector(const cv::Point3d& choosen, 
                                                       int radius)
{
    vector<cv::Point3d> dots;
    std::copy_if(_dots.begin(), _dots.end(), std::back_inserter(dots), 
        [choosen, radius](const Point3d& p) {
        return norm(choosen - p) <= radius && p != choosen;
    });
    return dots;
}

vector<Point3d> CylinderCloud::tube_to_vector()
{
    vector<Point3d> points;
    for (const auto& point : _tube) points.emplace_back(point);
    return points;
}

cv::Affine3d CylinderCloud::get_affine(const Point3d& point)
{
    // SCALE -> ROTATION -> TRANSLATION !
    //cout << "eigen : \n" << _T.matrix() << endl;
    //Eigen::AlignedScaling3d(5, 5, 5); // radius
    auto T = Eigen::Translation3d(point.x, point.y, point.z)
        * _T.rotation();
    auto affine3d = Affine3d();
    for (u_int i = 0; i < 4; ++i)
    {
        for (u_int j = 0; j < 4; ++j)
        {
            affine3d.matrix(i, j) = T.matrix().row(i)(j);
        }
    }
    //cout << "opencv : \n" << affine3d.matrix << endl;
    
    return affine3d;
}