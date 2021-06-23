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
#include <Eigen/Geometry> 


// max noise percentage for cloud dots
constexpr double MAX_CLOUD_DOTS_NOISE = 20. / 100;

constexpr unsigned int MIN_HEIGHT = 20;
constexpr unsigned int MAX_HEIGHT = 60;

// max points for cloud points
constexpr unsigned int DOTS_THRESHOLD = 200;


class CylinderCloud
{
    public:
        CylinderCloud (const cv::Point3d& origin)
            : _origin { origin }
        {
            generate_random_cloud();
        }

        /**
         * @brief generate a random cylinder dot cloud with random size,
         * rotation and position
         */
        CylinderCloud () : CylinderCloud { random_origin() }
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
         * @brief opencv display red dots if distance < radius
         * @arg cam : the opencv, viz camera (window)
         * @arg p : a selected point
         * @arg radius : the radius of neighbourhood
         */
        void display(cv::viz::Viz3d& cam, const cv::Point3d& p, int radius);


        /**
         * @brief get the nearest point from 0, 0, z
         */
        cv::Point3d centered_p();

        /**
         * @brief Transform the array container to the same vector container
         * @arg p : a selected point
         * @arg radius : the radius of neighbourhood
         * @return the list of dots which are nearest than radius
         */
        std::vector<cv::Point3d> dots_to_vector(const cv::Point3d& choosen, int radius);


        std::vector<cv::Point3d> _dots;



    private:

        /**
         * @brief generate a random 3D point for _origin
         * @return an Opencv 3D random point
         */
        cv::Point3d random_origin();

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

        /**
         * @brief generate the translation * rotation matrix _T :
         * the transformation matrix
         */
        void generate_transformation_mat();

        /**
         * @brief compute for each dot of _face1, _face2 and _tube
         * the transformation matrix : dot = _T * dot
         */
        void compute_transformation_matrix();

        /**
         * @brief compute the transformation matrix for a dot
         * @arg dot : dot to transform
         * @return the transformed dot
         */
        cv::Point3d compute_transformation_matrix(const cv::Point3d& dot);

        /**
         * @brief Transform the array container to the same vector container
         */
        void dots_to_vector();


        std::array<cv::Point3d, DOTS_THRESHOLD * 2> _face1;
        std::array<cv::Point3d, DOTS_THRESHOLD * 2> _face2;
        std::array<cv::Point3d, DOTS_THRESHOLD * 4> _tube;

        cv::Point3d _origin;
        cv::Vec3d   _vec;

        Eigen::Transform<double, 3, Eigen::Affine> _T;

        double _height = generateUniformDouble(MIN_HEIGHT, MAX_HEIGHT);
};