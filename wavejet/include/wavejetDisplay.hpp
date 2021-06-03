#pragma once

#include <Eigen/Core>

#include <math.h>
#include <complex>
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/viz/viz3d.hpp>

#include "phi.hpp"

constexpr unsigned int ANGLE_DIVISION  = 64;
constexpr unsigned int RADIUS_DIVISION = 12;
constexpr unsigned int WAVEJET_DISPLAY_RADIUS = 3;


class WavejetDisplay
{
    public:
        WavejetDisplay (const Phi& phi, u_int order)
                    : _phi   { phi },
                      _order { order }
        { }

        /**
         * find all position in R3 (x, y, z) to draw get points between 
         * WAVEJET_DISPLAY_RADIUS and ANGLE_DIVISION
         */
        void compute_and_find_point_position();

        /**
         * diplay on opencv window circle dots and triangle to
         * render the display 
         * @arg cam : the opencv camera
         */
        void display (cv::viz::Viz3d& cam);


    private:
        
        /**
         * draw the first radius circle of wavejet render (as triangle)
         * @arg cam : the opencv camera
         */
        void display_1st_radius (cv::viz::Viz3d& cam);

        /**
         * draw all others radius circle of wavejet render (as quadrilateral)
         * @arg cam : the opencv camera
         */
        void display_all_radius (cv::viz::Viz3d& cam);

        /**
         * compute f according to function 2.15 in the thesis
         * @arg r     : the point radius
         * @arg theta : the point angle
         */
        double f(double r, double theta);
        

        Phi   _phi;
        u_int _order;

        
        std::vector<cv::Point3d> _f; 
};