#include <iostream>

#include "wavejet.hpp"
#include "CylinderCloud.hpp"

#include "../../tests/randomizer.hpp"

#include <opencv2/viz/viz3d.hpp>
#include <Eigen/Core>


using namespace std;
using namespace cv;
using namespace viz;


// wavejet order demo
constexpr unsigned int ORDER = 2;
constexpr double NEIGHBOURHOOD_RADIUS = 2.;


Viz3d init_window()
{
    Viz3d cam { "Wavejets Demo" };
    cam.setBackgroundColor(Color::white());
    cam.showWidget("coordinate", WCoordinateSystem(10));
    return cam;
}

void display([[maybe_unused]] Viz3d& cam, 
             [[maybe_unused]] Wavejet& wj,
             [[maybe_unused]] CylinderCloud& cylDots,
             [[maybe_unused]] u_int idx,
             [[maybe_unused]] Affine3d transform,
             Point3d point)
{
    wj.display_svdV(cam);
    wj.display(cam, transform, idx);
    cylDots.display(cam);
    cylDots.display(cam, point, NEIGHBOURHOOD_RADIUS);
}

int main(int argc, char** argv)
{
    if (argc != 1)
    {
        cout << "wrong args : " << argc << argv << endl;
    }
    auto cam = init_window();

    CylinderCloud cylDots;

    u_int cpt = 0;
    for (const auto& point : cylDots.tube_to_vector())
    {
        cout << "treating ... " << cpt << endl;
        Wavejet wj {ORDER, 
                    point, 
                    cylDots.dots_to_vector(point, NEIGHBOURHOOD_RADIUS), 
                    NEIGHBOURHOOD_RADIUS};
        
        wj._phi.prompt_display();
        
        display(cam, wj, cylDots, cpt, wj.get_svd(), point);
        cpt ++;
        if (cpt >= 30) break; // too long else ...
    }
    
    cam.spin();

    return 0;
}
