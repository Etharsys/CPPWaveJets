#include <iostream>

#include "wavejet.hpp"
#include "CloudDots.hpp"

#include <opencv2/viz/viz3d.hpp>
#include <Eigen/Core>


using namespace std;
using namespace cv;
using namespace viz;


Viz3d init_window()
{
    Viz3d cam { "Wavejets Demo" };
    cam.setBackgroundColor(Color::white());
    cam.showWidget("coordinate", WCoordinateSystem(20));
    return cam;
}

void display(Viz3d& cam, CloudDots& cd, Wavejet<ORDER>& wj)
{
    cd.display(cam);
    wj.display_svdV(cam);
}

int main(int argc, char** argv)
{
    if (argc != 1)
    {
        cout << "wrong args : " << argc << argv << endl;
    }
    auto cam = init_window();

    CloudDots cd;
    Wavejet<ORDER> wj { cd.centered_p(), cd._dots, 100.};

    display(cam, cd, wj);

    cam.spin();

    return 0;
}



/*Wavejet<2> wj2 { {1, 2, -1}, , 30.};
    cout << wj << endl;*/