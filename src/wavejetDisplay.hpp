#pragma once

#include <Eigen/Core>

#include <math.h>
#include <complex>
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/viz/viz3d.hpp>

#include "config.hpp"
#include "phi.hpp"


template<u_int Order>
class WavejetDisplay
{
    public:
        WavejetDisplay (const Phi<Order>& phi)
                    : _phi { phi }
        {
            _phi.set(0, 0, std::complex<double> { 0, 0 });
            _phi.set(1,-1, std::complex<double> { 0, 0 });
            _phi.set(1, 1, std::complex<double> { 0, 0 });
            _phi.set(2,-2, std::complex<double> { 0, 0 });
            _phi.set(2, 0, std::complex<double> { 0, 0 });
            _phi.set(2, 2, std::complex<double> { 0.1, 0.1 });
        }

        void compute_display()
        {
            compute_f();
        }

        void display (cv::viz::Viz3d& cam)
        {
            cam.showWidget("dots", cv::viz::WCloud(_f, cv::viz::Color::black()));
            for (u_int i = 1; i <= ANGLE_DIVISION; ++i)
            { // draw the first radius
                auto j = (i != ANGLE_DIVISION ? i+1 : 1);
                std::vector<cv::Point3d> triangle { _f.at(0), _f.at(i), _f.at(j) };
                cv::Mat polygon3 = (cv::Mat_<int>(1,4) << 3, 0, 1, 2);
                std::string name = "triangle" + std::to_string(i);
                cam.showWidget(name, cv::viz::WMesh { triangle, polygon3 });
            }
            u_int i = ANGLE_DIVISION;
            for (u_int r = 1; r < RADIUS_DIVISION - 1; ++r)
            {
                for (u_int t = ANGLE_DIVISION; t < ANGLE_DIVISION*2; ++t)
                {
                    //cout << r << ", " << t << endl;
                    i++;
                    auto j = (i % ANGLE_DIVISION != 0  ? i+1 : i - ANGLE_DIVISION + 1);
                    /*cout << i << ", " << j << ", " <<  i-ANGLE_DIVISION << ", " 
                    << j-ANGLE_DIVISION << ", " << _f.at(i) << _f.at(j) <<
                    _f.at(i - ANGLE_DIVISION) << _f.at(j - ANGLE_DIVISION) << endl;*/
                    std::vector<cv::Point3d> paral { _f.at(i), 
                                                    _f.at(j), 
                                                    _f.at(i - ANGLE_DIVISION),
                                                    _f.at(j - ANGLE_DIVISION) };
                    cv::Mat polygon4 = (cv::Mat_<int>(1,5) << 4, 0, 1, 2, 3);
                    std::string name = "paral" + std::to_string(i);
                    cam.showWidget(name, cv::viz::WMesh { paral, polygon4 });
                }
            }
            cam.showWidget("d01", cv::viz::WCloud(
                std::vector { _f.at( 1) }, cv::viz::Color::yellow()));
            cam.showWidget("d25", cv::viz::WCloud(
                std::vector { _f.at(25) }, cv::viz::Color::red()));
            cam.showWidget("d24", cv::viz::WCloud(
                std::vector { _f.at(24) }, cv::viz::Color::green()));
        }


    private:
        
        void compute_f()
        {
            _f.emplace_back(0, 0, compute_r_theta_f(0, 0));
            for (double r = double(WAVEJET_DISPLAY_RADIUS) / RADIUS_DIVISION
                    ; r < WAVEJET_DISPLAY_RADIUS
                    ; r += double(WAVEJET_DISPLAY_RADIUS) / RADIUS_DIVISION)
            {
                for (double theta = 2 * M_PI / ANGLE_DIVISION
                    ; theta < 2 * M_PI
                    ; theta += 2 * M_PI / ANGLE_DIVISION)
                {
                    auto z = compute_r_theta_f(r, theta);
                    _f.emplace_back(r*cos(theta), r*sin(theta), z);
                }
            }
        }

        double compute_r_theta_f(double r, double theta)
        {
            std::complex<double> complex_f = 0;
            u_int idx = 0;
            for (u_int k = 0; k <= Order; ++k) 
            {
                //cout << "r : " << r << " -> " << rk << endl;
                for (int n = -int(k); n <= int(k); n+=2) 
                {
                    std::complex<double> e { cos(n * theta), sin(n * theta) };

                    //cout << "idx : " << idx << " = " << k << ", " << n << endl;
                    complex_f += _phi.at(k,n) * pow(r, k) * e;
                    idx++;
                }
            }
            //return abs(complex_f);
            //return sqrt(pow(complex_f.real(), 2) + pow(complex_f.imag(), 2));
            return complex_f.real();
        }
        

        Phi<Order> _phi;

        //Eigen::VectorXd  _f { ANGLE_DIVISION * RADIUS_DIVISION };
        std::vector<cv::Point3d> _f; 
};