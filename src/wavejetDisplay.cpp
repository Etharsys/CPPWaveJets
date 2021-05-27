#include "wavejetDisplay.hpp"


using namespace std;
using namespace cv;
using namespace viz;

void WavejetDisplay::compute_display()
{
    compute_f();

}

void WavejetDisplay::display([[maybe_unused]]cv::viz::Viz3d& cam)
{
    cam.showWidget("dots", WCloud(_f, Color::black()));
}

void WavejetDisplay::compute_f()
{
    for (double t = 0; t < WAVEJET_DISPLAY_RADIUS
                     ; t += WAVEJET_DISPLAY_RADIUS / RADIUS_DIVISION)
    {
        for (double r = 0; r < 2 * M_PI; r += 2 * M_PI / ANGLE_DIVISION)
        {
            [[maybe_unused]]auto z = compute_r_theta_f(r, t);
            cout << t << ", " << r, << ", " << z << endl;
            _f.emplace_back(t*cos(r), t*sin(r), z);
        }
    }
}

double WavejetDisplay::compute_r_theta_f(double r, double theta)
{
    complex<double> complex_f = 0;
    u_int idx = 0;
    for (u_int k = 0; k <= _k; ++k) 
    {
        auto rk = pow(r, k);
        for (int n = -int(k); n <= int(k); n+=2) 
        {
            double n_thet = n * theta;
            complex<double> e { cos(n_thet), sin(n_thet) };

            //complex_f += _phi(idx) * rk * e;
            complex_f += _phi(idx) * rk * e;
            idx++;
        }
    }
    return sqrt(pow(complex_f.real(), 2) + pow(complex_f.imag(), 2));
}