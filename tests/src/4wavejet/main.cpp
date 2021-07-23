#include <iostream>

#include <opencv2/viz/viz3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <complex>
#include <vector>
#include <math.h>

#include "phi.hpp"
#include "wavejetDisplay.hpp"


using namespace std;
using namespace cv;
using namespace viz;


Viz3d init_window()
{
    Viz3d cam { "Wavejets Demo" };
    cam.setBackgroundColor(Color::white());
    return cam;
}


int main(int argc, char** argv)
{
    if (argc != 1)
    {
        cout << "wrong args : " << argc << argv << endl;
    }

    auto cam = init_window();
    
    int K = 2;
    Phi phi { u_int(K) };
    phi.of_zeros();

    phi.wiseset(2, 2, complex<double> { 0.05, 0 });
    phi.wiseset(2, 0, complex<double> { 0.1, 0 });

    WavejetDisplay wd { phi, u_int(K), 1. };
    wd.compute_and_find_point_position();
    wd.display(cam); 

    cam.spin();

    return 0;
}