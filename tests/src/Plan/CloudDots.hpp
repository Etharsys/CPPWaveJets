#pragma once

#include "../../tests/randomizer.hpp"

#include <array>
#include <cassert>
#include <iostream>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz/viz3d.hpp>

#include <Eigen/Dense>


// max noise percentage for cloud dots
constexpr double MAX_CLOUD_DOTS_NOISE = 20. / 100;

constexpr int MAX_CLOUD_POINTS = 1000;


class CloudDots
{
    public:
        CloudDots (const cv::Point3d& origin, 
                   const cv::Vec3d& vecA, 
                   const cv::Vec3d& vecB)
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
            while (_vecA == _vecB) // faux : -collinear
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


        std::vector<cv::Point3d> dots_to_vector();




    private:

        /**
         * @brief find an orthogonal vector of A in the plane vecA-vecB
         * then normalize vectors A and B
         */
        void make_vector_square();

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


        std::array<cv::Point3d, MAX_CLOUD_POINTS> _dots;

        cv::Point3d _origin; 
        cv::Vec3d   _vecA;
        cv::Vec3d   _vecB;
        cv::Vec3d   _colA;

};