#pragma once

#include "config.hpp"
#include <array>
#include <stdlib.h>
#include <cassert>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz/viz3d.hpp>


class CloudDots
{
    public:
        CloudDots (cv::Point3d origin, cv::Vec3d vecA, cv::Vec3d vecB)
            : _origin { origin }, 
              _vecA   { vecA }, 
              _vecB   { vecB }
        {
            assert(vecA != vecB);
            generate_random_cloud();
        }

        CloudDots () : CloudDots { random_origin(), 
                                   random_3d_vector(),
                                   random_3d_vector() }
        {
            while (_vecA == _vecB)
            {
                _vecB = random_3d_vector();
            }
        }

        /**
         * @brief generate 2D randoms points on the plan then noise them
         */
        void generate_random_cloud();

        /**
         * @brief opencv display
         * @arg cam : the opencv, viz camera (window)
         */
        void display(cv::viz::Viz3d& cam);

        /**
         * @brief get the nearest point from 0, 0, z
         */
        cv::Point3d centered_p();


        std::array<cv::Point3d, MAX_CLOUD_POINTS> _dots;


    private:

        /**
         * @brief generate a random 3D point for _origin
         * @return an Opencv 3D random point
         */
        cv::Point3d random_origin();

        /**
         * @brief generate a random 3D vector for _vecA or _vecB
         * @return an Opencv 3D vector
         */
        cv::Vec3d random_3d_vector();

        /**
         * @brief generate a 3D random point wrt the plan
         * @return an Opencv 3D point
         */
        cv::Point3d create_random_point_on_plan();

        /**
         * @brief generate all 3D points wrt the plan
         */
        void create_all_random_points_on_plan();

        /**
         * @brief apply a random noise on third coordinate of all dots
         */
        void generate_random_noise();


        cv::Point3d _origin; 
        cv::Vec3d   _vecA;
        cv::Vec3d   _vecB;

};