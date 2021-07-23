#pragma once

#include <Eigen/Core>

#include <math.h>
#include <complex>
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/viz/viz3d.hpp>

#include "phi.hpp"

// number of angle division for wavejet render
constexpr unsigned int ANGLE_DIVISION  = 20; // 64
// number of radius division for wavejet render
constexpr unsigned int RADIUS_DIVISION = 3;  //12


class WavejetDisplay
{
    public:
        WavejetDisplay (const Phi& phi, u_int order, double radius)
                    : _phi   { phi },
                      _order { order },
                      _radius { radius }
        { }

        /**
         * find all position in R3 (x, y, z) to draw get points between 
         * WAVEJET_DISPLAY_RADIUS and ANGLE_DIVISION
         */
        void compute_and_find_point_position();

        /**
         * diplay on opencv window circle points and triangles to
         * render the display 
         * @arg cam : the opencv camera
         * @arg transform : the opencv affine3d transformation
         * @arg idx : wavejet id
         */
        void display (cv::viz::Viz3d& cam, 
                      const cv::Affine3d& transform, 
                      u_int idx = 0);
        
        /**
         * @brief display with opencv the wavejet
         */
        void display(cv::viz::Viz3d& cam, u_int idx = 0);


    private:
        
        /**
         * draw the first radius circle of wavejet render (as triangles)
         */
        void display_1st_radius (cv::viz::Viz3d& cam, 
                                 const cv::Affine3d& transform, 
                                 u_int idx = 0);

        /**
         * draw all others radius circle of wavejet render (as quadrilaterals)
         */
        void display_all_radius (cv::viz::Viz3d& cam, 
                                 const cv::Affine3d& transform, 
                                 u_int idx = 0);

        /**
         * compute f according to function 2.15 in the thesis
         * @arg r     : the point radius
         * @arg theta : the point angle
         */
        double f(double r, double theta);
        

        Phi   _phi;
        u_int _order;

        double _radius;

        
        std::vector<cv::Point3d> _f; 
};