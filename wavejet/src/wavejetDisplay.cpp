#include "wavejetDisplay.hpp"


using namespace std;
using namespace cv;
using namespace viz;



void WavejetDisplay::display (Viz3d& cam, 
                              const Affine3d& transform, 
                              u_int idx)
{
    //cam.showWidget("wavejet points", WCloud(_f, Color::black()));
    display_1st_radius(cam, transform, idx);
    display_all_radius(cam, transform, idx);
}

void WavejetDisplay::display(cv::viz::Viz3d& cam, u_int idx)
{
    display(cam, cv::Affine3d(), idx);
}

void WavejetDisplay::display_1st_radius (Viz3d& cam, 
                                         const Affine3d& transform, 
                                         u_int idx)
{
    for (u_int i = 1; i <= ANGLE_DIVISION; ++i)
    { // draw the first radius
        auto j = (i != ANGLE_DIVISION ? i+1 : 1);
        
        vector<Point3d> triangle { _f.at(0), _f.at(i), _f.at(j) };
        Mat polygon3 = (Mat_<int>(1,4) << 3, 0, 1, 2);
        string name = to_string(idx) + "triangle" + to_string(i);
        cam.showWidget(name, WMesh { triangle, polygon3 }, transform);
    }
}

void WavejetDisplay::display_all_radius(Viz3d& cam, 
                                        const cv::Affine3d& transform, 
                                        u_int idx)
{
    u_int i = ANGLE_DIVISION;
    //cout << _f.size() << endl;
    for (u_int r = 1; r < RADIUS_DIVISION; ++r)
    { // draw other radius
        for (u_int t = ANGLE_DIVISION; t < ANGLE_DIVISION*2; ++t)
        {
            i++;
            auto j = (i % ANGLE_DIVISION != 0  ? i+1 : i - ANGLE_DIVISION + 1);
            //cout << i << ", " << j << endl;
            vector<Point3d> paral { _f.at(i), 
                                    _f.at(j), 
                                    _f.at(i - ANGLE_DIVISION),
                                    _f.at(j - ANGLE_DIVISION) };
            Mat polygon4 = (Mat_<int>(1,5) << 4, 0, 1, 2, 3);
            string name = to_string(idx) + "paral" + to_string(i);
            cam.showWidget(name, WMesh { paral, polygon4 }, transform);
        }
    }
}


void WavejetDisplay::compute_and_find_point_position()
{
    _f.emplace_back(0, 0, f(0, 0));
    for (double r2 = 1
            ; r2 <= RADIUS_DIVISION
            ; r2 ++)
    {
        auto r = r2 / RADIUS_DIVISION;
        for (double theta = 2 * M_PI / double (ANGLE_DIVISION)
            ; theta <= 2 * M_PI + (2 * M_PI / double(ANGLE_DIVISION * 2))
            ; theta += 2 * M_PI / double(ANGLE_DIVISION))
        {
            _f.emplace_back(r*cos(theta), r*sin(theta), f(r, theta));
        }
    }
    //cout << _f.size() << ", " << ANGLE_DIVISION * (RADIUS_DIVISION) + 1 << endl;
    assert(_f.size() == ANGLE_DIVISION * (RADIUS_DIVISION) + 1);
}


double WavejetDisplay::f(double r, double theta)
{
    complex<double> complex_f = 0;
    u_int idx = 0;
    for (u_int k = 0; k <= _order; ++k) 
    {
        //cout << "r : " << r << " -> " << rk << endl;
        for (int n = -int(k); n <= int(k); n+=2) 
        {
            complex<double> e { cos(n * theta), sin(n * theta) };

            //cout << "idx : " << idx << " = " << k << ", " << n << endl;
            complex_f += _phi.at(k,n) * pow(r, k) * e;
            idx++;
        }
    }
    //return abs(complex_f);
    return complex_f.real();
}