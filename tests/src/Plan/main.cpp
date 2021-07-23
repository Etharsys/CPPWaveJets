#include <iostream>

#include "wavejet.hpp"
#include "PlanCloud.hpp"

#include <opencv2/viz/viz3d.hpp>
#include <Eigen/Core>


using namespace std;
using namespace cv;
using namespace viz;


// wavejet order demo
constexpr unsigned int ORDER = 2;
constexpr double NEIGHBOURHOOD_RADIUS = 100.;


Viz3d init_window()
{
    Viz3d cam { "Wavejets Demo" };
    cam.setBackgroundColor(Color::white());
    cam.showWidget("coordinate", WCoordinateSystem(10));
    return cam;
}

void display([[maybe_unused]] Viz3d& cam, 
             [[maybe_unused]] Wavejet& wj,
             [[maybe_unused]] CloudPoints& cd)
{
    wj.display_svdV(cam);
    wj.display(cam);
    cd.display(cam);
}

int main(int argc, char** argv)
{
    if (argc != 1)
    {
        cout << "wrong args : " << argc << argv << endl;
    }
    auto cam = init_window();

    CloudPoints cd;
    Wavejet wj { ORDER, 
                 cd.centered_p(), 
                 cd.points_to_vector(), 
                 NEIGHBOURHOOD_RADIUS};

    //wj._phi.wiseset(0, 0, std::complex<double> { 0, 0 });
    //wj._phi.wiseset(1, 1, std::complex<double> { 0, 0 });
    wj._phi.prompt_display();
     

    display(cam, wj, cd);

    cam.spin();

    return 0;
}
