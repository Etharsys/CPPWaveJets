#pragma once

#include <cassert>
#include <iostream>
#include <math.h>
#include <complex>

#include <opencv2/core/core.hpp>
#include <opencv2/viz/viz3d.hpp>


#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/IterativeLinearSolvers>

#include "config.hpp"


using Neighbors = std::array<cv::Point3d, MAX_CLOUD_POINTS>;


template <unsigned int Order>
class Wavejet
{
    public :
        Wavejet (const cv::Point3d& p, 
                 const Neighbors& neighbors, 
                 double neighbor_radius)
            : _p { Eigen::Vector3d { p.x, p.y, p.z } }, 
              _neighbors { array_to_matrixXd(neighbors) }, 
              _nr { neighbor_radius }
        {
            assert(neighbor_radius >= 0);
            compute_wavejets();
        }

        /**
         * @brief compute the Order-Wavejets (until computing of phi)
         */
        void compute_wavejets()
        {
            set_lines_cols();
            neighbours_principal_vector();

            //switch_polar_coords(neighbours_coords(svdV));
            //compute_phi();
        }

        void display_svdV(cv::viz::Viz3d& cam)
        {
            cv::Point3d ori { _p(0) , _p(1) , _p(2) };
            cam.showWidget("origin", cv::viz::WCloud(std::vector {ori}, cv::viz::Color::green()));
            cv::Point3d t1  { _t1(0) * 10, _t1(1) * 10, _t1(2) * 10};
            cam.showWidget("t1", cv::viz::WLine(ori, t1 + ori, cv::viz::Color::blue()));
            cv::Point3d t2  { _t2(0) * 10, _t2(1) * 10, _t2(2) * 10};
            cam.showWidget("t2", cv::viz::WLine(ori, t2 + ori, cv::viz::Color::blue()));
            cv::Point3d nm  { _normal(0) * 10, _normal(1) * 10, _normal(2) * 10};
            cam.showWidget("normal", cv::viz::WLine(ori, nm + ori, cv::viz::Color::red()));
        }

        std::ostream& operator<< (std::ostream& stream)
        {
            return stream << Order << "-Wavejets : " << std::endl << _phi;
        }


    private :

        Eigen::MatrixXd array_to_matrixXd(const Neighbors& neighbors)
        {
            u_int i = 0;
            Eigen::MatrixXd mat { 3, MAX_CLOUD_POINTS };
            for (const auto& p : neighbors)
            {
                mat(0, i) = p.x;
                mat(1, i) = p.y;
                mat(2, i) = p.z;
                i++;
            }
            return mat;
        }

        /**
         * @brief set nneigh (number of neighbors) and ncolPhi (wrt Order) 
         */
        void set_lines_cols()
        {
            _nneigh  = _neighbors.row(0).size();
            _ncolPhi = (pow(Order, 2) / 2) + (3 * Order / 2) + 1;
        }

        /**
         * @brief Compute PCA (Principal Componant Analysis) normal, 
         * then get the last SVD (Singular Value Decomposition) 3D matrix
         * to the three principal neighbors vectors (as column) : 
         * [length, width, height=normal]
         * @return the svd V matrix
         */
        Eigen::Matrix3d neighbours_principal_vector()
        {
            Eigen::Vector3d average_p { _neighbors.row(0).mean(), 
                                        _neighbors.row(1).mean(),
                                        _neighbors.row(2).mean() };
            Eigen::Matrix3d c_matrix = 1./_nneigh * 
                                (_neighbors * _neighbors.transpose()) -
                                ( average_p *  average_p.transpose());
            Eigen::JacobiSVD<Eigen::MatrixXd> svd { c_matrix, Eigen::ComputeFullV };
            Eigen::Matrix3d v = svd.matrixV();
            _t1     = v.col(0);
            _t2     = v.col(1);
            _normal = v.col(2);
            return c_matrix;
        }

        /**
         * @brief get the cartesian coordinates of neighbors
         * @arg neighbors_principal_vectors : the neighbors dots set principal vectors
         * @return an array of all points in cartesian coordinates : (matrix:_nneigh x 3)
         */
        Eigen::MatrixXd neighbours_coords(const Eigen::Matrix3d& neighbors_principal_vectors) // rename & const&
        {
            Eigen::Matrix3d repp { _nneigh, 1 };
            for (u_int i = 0; i < _nneigh; ++i)
            {
                repp.col(i) = _p; // _nneigh * _p (as colon)
            }
            return (_neighbors - repp) * neighbors_principal_vectors;
        }

        /**
         * @brief switch neighbors in coords system to polar system 
         * by setting _radius, _theta and _z attributs
         * @arg ln : locneighbors matrix, the coordinates of neighbors in cartesian plan
         */
        void switch_polar_coords(const Eigen::MatrixXd& ln)
        {
            for (u_int i = 0; i < _nneigh; ++i) // revoir les < / <=
            {
                _radius(i) = sqrt( pow(ln(i,0), 2) + pow(ln(i,1), 2) );
                _theta(i)  = atan2(ln(i,1), ln(i,0));
            }
            _z = ln.col(3);
        }

        /**
         * @brief compute the Order-Wavejet Phi matrix 
         * and fill _indices (used indices) matrix
         */
        void compute_phi()
        {
            /*Eigen::MatrixXd M       { _nneigh , _ncolPhi };
            Eigen::MatrixXd indices { _ncolPhi, 2 };
            unsigned int idx   = 1;
            double norm_radius = _radius / _nr;
            double norm_z      = _z      / _nr;
            auto   w           = exp(pow(-norm_radius, 2) / 18);
            for (u_int k = 0; k <= Order; ++k) 
            {
                auto rk = pow(norm_radius, k);
                for (int n = -k; n <= k; n+=2)
                {
                    const std::complex<double> i(0, 1);
                    double d_n = 1. * n;
                    for (int p = 0; p < _nneigh; ++p)
                    {
                        auto e = cos(d_n * _theta(p)) + i * sin(d_n * _theta(p));
                        M(idx, p) = rk * e * w;
                    }
                    Eigen::Vector2d v{ k, n };
                    indices.row(idx) = v;
                    idx++;
                }
            }
            Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Upper> solver;
            _phi = solver.compute(M).solve(_z * w);*/
        }
        
        

        unsigned int     _nneigh;        // number of neighbors
        unsigned int     _ncolPhi;       // number of Phi matrix columns

        Eigen::Vector3d  _p;             // entry point (1x3)
        Eigen::MatrixXd  _neighbors;     // neighbors dots (nx3)
        double           _nr;            // neighborhood radius (>0 !)

        Eigen::Vector3d  _t1;            // neighbors average length vector
        Eigen::Vector3d  _t2;            // neighbors average width vector
        Eigen::Vector3d  _normal;        // neighbors average normal

        Eigen::Vector3d  _radius;        // polar r (>0 !)
        Eigen::Vector3d  _theta;         // polar theta
        Eigen::Vector3d  _z;             // polar z

        Eigen::MatrixXcf _phi;           // phi matrix (_nneigh x _ncolPhi)
        Eigen::MatrixXd  _indices;       // used pair (k,n) in phi(k,n) (_ncolPhi x 2)
};