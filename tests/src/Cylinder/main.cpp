#include <iostream>

#include "wavejet.hpp"
#include "CylinderCloud.hpp"

#include <opencv2/viz/viz3d.hpp>
#include <Eigen/Core>


using namespace std;
using namespace cv;
using namespace viz;


// wavejet order demo
constexpr unsigned int ORDER = 5;


Viz3d init_window()
{
    Viz3d cam { "Wavejets Demo" };
    cam.setBackgroundColor(Color::white());
    cam.showWidget("coordinate", WCoordinateSystem(10));
    return cam;
}

void display([[maybe_unused]] Viz3d& cam, 
             [[maybe_unused]] Wavejet& wj,
             [[maybe_unused]] CylinderCloud& cylDots)
{
    wj.display_svdV(cam);
    //wj.display(cam);
    cylDots.display(cam);
}

int main(int argc, char** argv)
{
    if (argc != 1)
    {
        cout << "wrong args : " << argc << argv << endl;
    }
    auto cam = init_window();

    CylinderCloud cylDots;
    Wavejet wj { ORDER, 
                 cylDots.centered_p(), 
                 cylDots.dots_to_vector(), 
                 100.};

    //wj._phi.wiseset(2, 0, 0);
    wj._phi.wiseset(1,-1, 0);
    wj._phi.prompt_display();
     

    display(cam, wj, cylDots);

    cam.spin();

    return 0;
}
