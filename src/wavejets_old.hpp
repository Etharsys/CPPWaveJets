#pragma once

#include <iostream>
#include <complex>

#include <Eigen/Core>


/**
 * @brief : compute phi (below (2))
 * If k and n does not share the same parity the coefficient will be 0
 * @arg k : integer (can be negativ)
 * @arg n : integer (can be negativ)
 */
std::complex<double> phi(int k, int n);

/**
 * @brief : compute b(k, j, n)
 * If k and n does not share the same parity : return 0
 * @arg k : same as phi 
 * @arg j : sum (0 to k) so can only be positiv
 * @arg n : same as phi
 */
std::complex<double> b(int k, int j, int n);

/**
 * @brief : reference to the article formula (2)
 * compute f using poral coordinates : (r, theta)
 * @arg r     : radius
 * @arg theta : angle
 */ 
std::complex<double> f(double r, double theta);


/* === Affichage === */

void display_4_wj();

class Wavejet
{
    public :
        Wavejet (int k, int n)
            : _k { k }, _n { n }, _phi { phi(k, n) }
        {}

        int _k;
        int _n;
        std::complex<double> _phi;
};