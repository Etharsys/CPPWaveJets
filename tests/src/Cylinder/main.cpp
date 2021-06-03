#include <iostream>

#include "wavejet.hpp"
#include "CloudDots.hpp"
#include "CylinderCloud.hpp"

#include <opencv2/viz/viz3d.hpp>
#include <Eigen/Core>


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

void display([[maybe_unused]] Viz3d& cam, 
             [[maybe_unused]] CloudDots& cd, 
             [[maybe_unused]] Wavejet& wj,
             [[maybe_unused]] CylinderCloud& cylDots)
{
    //cd.display(cam);
    //wj.display_svdV(cam);
    wj.display(cam);
    //cylDots.display(cam);
}

int main(int argc, char** argv)
{
    if (argc != 1)
    {
        cout << "wrong args : " << argc << argv << endl;
    }
    auto cam = init_window();

    CloudDots cd;
    //Wavejet<ORDER> wj { cd.centered_p(), cd._dots, 100.};
    CylinderCloud cylDots;
    Wavejet wj { ORDER, 
                 cylDots.centered_p(), 
                 cylDots.dots_to_vector(), 
                 100.};

    display(cam, cd, wj, cylDots);

    cam.spin();

    return 0;
}
