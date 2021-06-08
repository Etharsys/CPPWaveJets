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


Viz3d init_window();
void update();
void KeyboardViz3d(const viz::KeyboardEvent &w, void *t);


int K = 0;
int N = 0;
auto cam = init_window();


Viz3d init_window()
{
    Viz3d cam { "Wavejets Demo" };
    cam.setBackgroundColor(Color::white());
    return cam;
}


void update()
{
    Phi phi { u_int(K) };
    phi.of_zeros();

    auto scale = K == 0 ? 1 : exp(K) / (K * 2);
    phi.wiseset(K, N, complex<double> { 0.1 / (scale), 0.1 / (scale) });

    WavejetDisplay wd { phi, u_int(K) };
    wd.compute_and_find_point_position();
    wd.display(cam);   
}

void KeyboardViz3d(const viz::KeyboardEvent &w, void *t)
{
    viz::Viz3d *fen=(viz::Viz3d*)t;
    if (w.action)
    {
        switch(w.code)
        {
            case 'p' :
                K = K == 10 ? 10 : K+1;
                N=-K;
                break;
            case 'o' :
                K = K == 0 ? 0 : K-1;
                N=-K;
                break;
            case 'm' :
                N = N == K ? K : N+2;
                break;
            case 'l' :
                N = N == -K ? -K : N-2;
                break;
            default  :
                return;
        }
        cout << fen->getWindowName() << " --- (" << K << "," << N << ")" << endl;
        update();
    }
}

int main(int argc, char** argv)
{
    if (argc != 1)
    {
        cout << "wrong args : " << argc << argv << endl;
    }

    cam.registerKeyboardCallback(KeyboardViz3d,&cam);
    update();

    cam.spin();

    return 0;
}