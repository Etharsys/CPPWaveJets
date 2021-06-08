#pragma once

#include "../../tests/randomizer.hpp"

#include <array>
#include <cassert>
#include <iostream>
#include <math.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz/viz3d.hpp>

#include <Eigen/Dense>

// max points for cloud points
constexpr unsigned int MAX_CLOUD_POINTS   = 200;

constexpr unsigned int MAX_CYLINDER_CLOUD = 
    MAX_CLOUD_POINTS * 2 + // 2 faces
    MAX_CLOUD_POINTS * 4;  // cyl


class CylinderCloud
{
    public:
        CylinderCloud (const cv::Point3d& origin, const cv::Vec3d& vec)
            : _origin { origin }, _vec { vec }
        {
            generate_random_cloud();
        }

        CylinderCloud () : CylinderCloud { random_origin(), random_3d_vector() }
        { }

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
        cv::Point3d create_random_point_on_plan(double z);

        /**
         * @brief generate a 3D random point wrt the cylinder 'tube'
         * @return an Opencv 3D point
         */
        cv::Point3d create_random_point_on_cyl();

        /**
         * @brief generate all 3D points wrt the plan
         */
        void create_all_random_points_on_plan();

        /**
         * @brief apply a random noise on third coordinate of all dots
         */
        void generate_random_noise();

        void apply_rotation();


        std::array<cv::Point3d, MAX_CYLINDER_CLOUD> _dots;

        cv::Point3d _origin;
        cv::Vec3d   _vec;

        double _height = generateUniformDouble(10, 30);

};