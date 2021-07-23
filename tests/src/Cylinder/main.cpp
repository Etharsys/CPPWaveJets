#include <iostream>

#include "wavejet.hpp"
#include "CylinderCloud.hpp"

#include "../../tests/randomizer.hpp"

#include <opencv2/viz/viz3d.hpp>
#include <Eigen/Core>


using namespace std;
using namespace cv;
using namespace viz;


// wavejet order
constexpr unsigned int ORDER = 2;
// 
constexpr double NEIGHBOURHOOD_RADIUS = 2.;


Viz3d init_window()
{
    Viz3d cam { "Wavejets Demo" };
    cam.setBackgroundColor(Color::white());
    //cam.showWidget("coordinate", WCoordinateSystem(10));
    return cam;
}

void display([[maybe_unused]] Viz3d& cam, 
             [[maybe_unused]] Wavejet& wj,
             [[maybe_unused]] CylinderCloud& cylPoints,
             [[maybe_unused]] u_int idx,
             [[maybe_unused]] const Affine3d& transform,
             [[maybe_unused]] const Point3d& point,
             [[maybe_unused]] int neighbourhood_radius)
{
    //wj.display_svdV(cam);
    wj.display(cam, transform, idx);
    //cylPoints.display(cam);
    //cylPoints.display(cam, point, neighbourhood_radius);
}

int get_variable(const string& msg, int min, int max)
{
    int var = -1;
    while (var < min || var > max)
    {
        cout << msg << "(" << min << "," << max << ") : ";
        cin >> var;
    }
    return var;
}

int main(int argc, char** argv)
{
    if (argc != 1)
    {
        cout << "wrong args : " << argc << argv << endl;
    }
    auto order = get_variable("Enter a wavejet order : ", 2, 10);
    auto neighbourhood_radius = get_variable("Enter a neighbourhood radius : ", 1, 10);
    auto cam = init_window();

    CylinderCloud cylPoints;

    u_int cpt = 0;
    for (const auto& point : cylPoints.tube_to_vector())
    {
        cout << "treating ... " << cpt << endl;
        Wavejet wj {u_int (order), 
                    point, 
                    cylPoints.points_to_vector(point, neighbourhood_radius), 
                    double (neighbourhood_radius) };
        
        //wj._phi.prompt_display();
        
        display(cam, wj, cylPoints, cpt, wj.get_svd(), point, neighbourhood_radius);
        cpt ++;
        if (cpt >= 70) break; // too long else ...
    }
    
    cam.spin();

    return 0;
}
