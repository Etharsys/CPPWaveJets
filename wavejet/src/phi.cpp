#include "phi.hpp"



std::complex<double> Phi::at (u_int k, int n)
{
    return _phi.at(k).at((k + n) / 2);
}


void Phi::set (u_int k, int n, const std::complex<double>& val)
{
    auto it = _phi.at(k).begin() + (k + n) / 2;
    if (_phi.at(k).size() <= (k + n) / 2)
    {
        _phi.at(k).emplace(it, val);
    } else 
    {
        _phi.at(k).at((k + n) / 2) = val;
    }
}

void Phi::wiseset (u_int k, int n, const std::complex<double>& val)
{
    set(k, n, val);
    if (n != 0)
    {
        set(k, -n, std::conj(val));
    }
}

void Phi::prompt_display()
{
    for (u_int k = 0; k <= _order; ++k) 
    {
        for (int n = -int(k); n <= int(k); n+=2) 
        {
            std::cout << "phi("<< k << "," << n << ") = " 
                      << at(k,n) << std::endl;
        }
    }
}

void Phi::of_zeros()
{
    for (u_int k = 0; k <= _order; ++k) 
    {
        for (int n = -int(k); n <= int(k); n+=2) 
        {
            set(k, n, 0);
        }
    }
}