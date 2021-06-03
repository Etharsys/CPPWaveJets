#include <iostream>

#include <opencv2/viz/viz3d.hpp>
#include <complex>

#include "phi.hpp"
#include "wavejetDisplay.hpp"


using namespace std;
using namespace cv;
using namespace viz;


Viz3d init_window()
{
    Viz3d cam { "Wavejets Demo" };
    cam.setBackgroundColor(Color::white());
    cam.showWidget("coordinate", WCoordinateSystem(10));
    return cam;
}

u_int get_order()
{
    u_int order = 0;
    while (order <= 1 || order >= 10)
    {
        cout << "Enter order [2,11] : ";
        cin >> order;
    }
    return order;
}

int main(int argc, char** argv)
{
    if (argc != 1)
    {
        cout << "wrong args : " << argc << argv << endl;
    }
    auto order = get_order();

    auto cam = init_window();

    Phi phi_t1 { order };

    phi_t1.set(0, 0, complex<double> { 0, 0 });
    phi_t1.set(1,-1, complex<double> { 0, 0 });
    phi_t1.set(1, 1, complex<double> { 0, 0 });
    phi_t1.set(2,-2, complex<double> { 0.1, 0.1 });
    phi_t1.set(2, 0, complex<double> { 0, 0 });
    phi_t1.set(2, 2, complex<double> { 0.1,-0.1 });

    WavejetDisplay wd { phi_t1, order };

    wd.compute_and_find_point_position();
    wd.display(cam);

    cam.spin();

    return 0;
}
