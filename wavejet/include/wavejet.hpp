#pragma once

#include <cassert>
#include <iostream>
#include <math.h>
#include <complex>
#include <typeinfo>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/viz/viz3d.hpp>


#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/LU>

#include "wavejetDisplay.hpp"
#include "phi.hpp"


using Neighbors = std::vector<cv::Point3d>;


class Wavejet
{
    public :
        Wavejet (u_int order,
                 const cv::Point3d& p, 
                 const Neighbors& neighbors, 
                 double neighbor_radius)
            : _order { order },
              _p { Eigen::Vector3d { p.x, p.y, p.z } }, 
              _neighbors { list_to_matrixXd(neighbors) }, 
              _nr { neighbor_radius }
        {
            assert(neighbor_radius >= 0);
            //std::cout << _p << std::endl;
            compute_wavejets();
        }

        /**
         * @brief compute the Order-Wavejets (until computing of phi)
         */
        void compute_wavejets();

        /**
         * @brief display with opencv the SVD vectors
         * @arg cam : the opencv camera
         */
        void display_svdV(cv::viz::Viz3d& cam);

        /**
         * @brief display with opencv the wavejet
         * @arg cam : the opencv camera
         */
        void display(cv::viz::Viz3d& cam);


        Phi _phi { _order };



    private :

        /**
         * @brief transform a vector of dots in a eigen matrix
         * @arg neighbors : the container of dots (as vector)
         * @return all dots as eigen matrix (row = dots and col = x,y,z)
         */
        Eigen::MatrixXd list_to_matrixXd(const Neighbors& neighbors);

        /**
         * @brief set nneigh (number of neighbors) and ncolPhi (wrt Order) 
         */
        void set_lines_cols();

        /**
         * @brief Compute PCA (Principal Componant Analysis) normal, 
         * then get the last SVD (Singular Value Decomposition) 3D matrix
         * to the three principal neighbors vectors (as column) : 
         * [length, width, height=normal]
         * @return the svd V matrix
         */
        Eigen::Matrix3d neighbours_principal_vector();

        /**
         * @brief get the cartesian coordinates of neighbors
         * @arg neighbors_principal_vectors : the neighbors dots set principal vectors
         * @return an array of all points in cartesian coordinates : (matrix:_nneigh x 3)
         */
        Eigen::MatrixXd neighbours_coords(const Eigen::Matrix3d& neighbors_principal_vectors);

        /**
         * @brief switch neighbors in coords system to polar system 
         * by setting _radius, _theta and _z attributs
         * @arg ln : locneighbors matrix, the coordinates of neighbors in cartesian plan
         */
        void switch_polar_coords(const Eigen::MatrixXd& locneighbors);

        /**
         * @brief compute the Order-Wavejet Phi matrix 
         * and fill _indices (used indices) matrix
         */
        void compute_phi();

        /**
         * @brief compute an() values
         */
        void compute_a();



        unsigned int     _order;         // wavejet order

        unsigned int     _nneigh;        // number of neighbors
        unsigned int     _ncolPhi;       // number of Phi matrix columns

        Eigen::Vector3d  _p;             // entry point (1x3)
        Eigen::MatrixXd  _neighbors;     // neighbors dots (nx3)
        double           _nr;            // neighborhood radius (>0 !)

        Eigen::Vector3d  _t1;            // neighbors average length vector
        Eigen::Vector3d  _t2;            // neighbors average width vector
        Eigen::Vector3d  _normal;        // neighbors average normal

        Eigen::VectorXd  _radius;        // polar r (>0 !)
        Eigen::VectorXd  _theta;         // polar theta
        Eigen::VectorXd  _z;             // polar z

        Eigen::MatrixXd  _indices;       // used pair (k,n) in phi(k,n) (_ncolPhi x 2)

        Eigen::MatrixXcd _an;            // an values
};