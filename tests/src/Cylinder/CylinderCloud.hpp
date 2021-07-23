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


// max noise percentage for cloud points
constexpr double MAX_CLOUD_POINTS_NOISE = 20. / 100;

constexpr unsigned int MIN_HEIGHT = 5;
constexpr unsigned int MAX_HEIGHT = 10;

// max points for cloud points
constexpr unsigned int POINTS_THRESHOLD = 1000;

// max size of cloud points plan
constexpr int MAX_POINTS_CLOUD_RADIUS = 10;


class CylinderCloud
{
    public:
        CylinderCloud (const cv::Point3d& origin)
            : _origin { origin }
        {
            generate_random_cloud();
        }

        /**
         * @brief generate a random cylinder point cloud with random size,
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
         * @brief opencv display red points if distance < radius
         * @arg cam : the opencv, viz camera (window)
         * @arg point : a selected point
         * @arg radius : the radius of neighbourhood
         */
        void display(cv::viz::Viz3d& cam, const cv::Point3d& point, int radius);


        /**
         * @brief get the nearest point from 0, 0, z
         */
        cv::Point3d centered_p();

        /**
         * @brief Transform the array container to the same vector container
         * @arg choosen : a selected point
         * @arg radius : the radius of neighbourhood
         * @return the list of points which are nearest than radius
         */
        std::vector<cv::Point3d> points_to_vector(const cv::Point3d& choosen, int radius);

        std::vector<cv::Point3d> tube_to_vector();

        cv::Affine3d get_affine(const cv::Point3d& point);


        std::vector<cv::Point3d> _points;



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
         * @brief apply a random noise on third coordinate of all points
         */
        void generate_random_noise();

        /**
         * @brief generate the translation * rotation matrix _T :
         * the transformation matrix
         */
        void generate_transformation_mat();

        /**
         * @brief compute for each point of _face1, _face2 and _tube
         * the transformation matrix : point = _T * point
         */
        void compute_transformation_matrix();

        /**
         * @brief compute the transformation matrix for a point
         * @arg point : point to transform
         * @return the transformed point
         */
        cv::Point3d compute_transformation_matrix(const cv::Point3d& point);

        /**
         * @brief Transform the array container to the same vector container
         */
        void points_to_vector();


        std::array<cv::Point3d, POINTS_THRESHOLD * 2> _face1;
        std::array<cv::Point3d, POINTS_THRESHOLD * 2> _face2;
        std::array<cv::Point3d, POINTS_THRESHOLD * 4> _tube;

        Eigen::Transform<double, 3, Eigen::Affine> _T;

        cv::Point3d _origin;
        cv::Vec3d   _vec;

        double _height = generateUniformDouble(MIN_HEIGHT, MAX_HEIGHT);
};