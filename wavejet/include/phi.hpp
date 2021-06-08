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

        /**
         * @brief get the value at phi(k, n)
         * @arg k : 1 st indice as k
         * @arg n : 2 nde indice as (k+n) / 2
         * @return phi(k, n) as a double
         */
        std::complex<double> at (u_int k, int n);

        /**
         * @brief add a new value if phi(k,n) does not exist or relace 
         * the existent
         * @arg k : 1 st indice as k
         * @arg n : 2 nde indice as (k+n) / 2
         * @arg val : the value to emplace
         */
        void set (u_int k, int n, const std::complex<double>& val);

        /**
         * @brief seeing that phi(k, n) = phi*(k,-n), we can set both
         * @arg k : 1 st indice as k
         * @arg n : 2 nde indice as (k+n) / 2
         * @arg val : the value to emplace
         */
        void wiseset (u_int k, int n, const std::complex<double>& val);

        /**
         * display the phi container on terminal
         */
        void prompt_display();

        /**
         * set all possible values of _phi(k, n) as 0
         */
        void of_zeros();


    private :
        u_int _order;

        /**
         * container des phi(k,n), n has no size set and k is 
         * order (K) + 1 (0 count)
         */
        std::vector<std::vector<std::complex<double>>> _phi { _order + 1 };



};