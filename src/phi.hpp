#pragma once

#include <complex>
#include <vector>
#include <array>

#include <Eigen/Core>

template <u_int Cardinal>
class Phi
{
    public :

        std::complex<double> at (u_int k, int n)
        {
            return _phi.at(k).at((k + n) / 2);
        }

        void set (u_int k, int n, const std::complex<double>& val)
        {
            auto it = _phi.at(k).begin() + (k + n) / 2;
            _phi.at(k).emplace(it, val);
            //std::cout << val << std::endl;
        }

        std::array<std::vector<std::complex<double>>, Cardinal + 1> _phi;

    private :

        int _cardinal;


};