#pragma once

#include <complex>
#include <vector>
#include <array>
#include <iostream>

#include <Eigen/Core>

class Phi
{
    public :

        Phi (u_int order)
            : _order { order }
        { }

        std::complex<double> at (u_int k, int n);

        void set (u_int k, int n, const std::complex<double>& val);

        /**
         * display the phi container on terminal
         */
        void prompt_display();


    private :
        u_int _order;

        /**
         * container des phi(k,n), n has no size set and k is 
         * order (K) + 1 (0 count)
         */
        std::vector<std::vector<std::complex<double>>> _phi { _order + 1 };



};