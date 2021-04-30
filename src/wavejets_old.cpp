#include "wavejets.hpp"

#include <iomanip>

#include "operation/misc.hpp"
#include "operation/factorial.hpp"

using namespace std;

complex<double> phi(int k, int n)
{
    complex<double> phi;
    for (int j = 0; j <= k; ++j)
    {
        cout << 1. / (facto(j) * facto(k - j)) * b(k,j,n) << endl;
        phi += 1. / (facto(j) * facto(k - j))
                 * b(k, j, n) 
                 * f(0, 0);
    }
    return phi;
}


complex<double> b(int k, int j, int n)
{
    using namespace complex_literals;
    auto result = 0. + 0.i;
    if (abs(k)%2 != abs(n)%2) // if not same parity : 0
    {
        return result;
    }

    result = 1. / (pow(2, k) * pow(1.i, j)); 

    for (int h = 0; h <= (n-k) / 2; ++h) // sum : 0 to n-k/2
    {
        result *= ubinomial(k-j, h) * 
                  ubinomial(j, (n-k)/2 - h) * 
                  pow(-1, h);
    }
    return result;
}

 
complex<double> f(double r, double theta)
{
    // TODO : check fnc polar from <complex> : compute for us ?
    if (false) {
        cout << r << theta << endl;
    }
    return 1.;
}


/* === Affichage === */

ostream& operator<< (ostream& stream, const Wavejet& wj)
{
    return stream << "phi(" << wj._k << "," << wj._n << ") = " << wj._phi;
}

void display_4_wj()
{
    /*
    for (int k = 0; k <= 2; ++k)
    {
        for (int j = -2; j <= 2; ++j)
        {
            Wavejet wj { k, j };
            cout << wj << endl;
        }
    }
    */
    Wavejet wj { 2, 0 };
    cout << wj << endl;
}