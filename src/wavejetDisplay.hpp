#pragma once

#include <Eigen/Core>

#include <math.h>
#include <complex>
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/viz/viz3d.hpp>

#include "config.hpp"


class WavejetDisplay
{
    public:
        WavejetDisplay (const Eigen::VectorXcd phi,
                        u_int K)
                    : _phi { phi },
                      _k   { K }
        { }

        void compute_display();

        void display (cv::viz::Viz3d& cam);


    private:
        
        void compute_f();

        double compute_r_theta_f(double r, double theta);
        
        void compute_triangle();


        Eigen::VectorXcd _phi;
        u_int _k;

        //Eigen::VectorXd  _f { ANGLE_DIVISION * RADIUS_DIVISION };
        std::vector<cv::Point3d> _f { ANGLE_DIVISION * RADIUS_DIVISION }; 
};