#include <iostream>

#include <opencv2/viz/viz3d.hpp>
#include <opencv2/imgcodecs.hpp>

#include <complex>
#include <vector>
#include <filesystem>
#include <math.h>

#include "phi.hpp"
#include "wavejetDisplay.hpp"


using namespace std;
using namespace cv;
using namespace viz;

namespace fs = std::filesystem;


Viz3d init_window()
{
    Viz3d cam { "Wavejets Demo" };
    cam.setBackgroundColor(Color::white());
    //cam.showWidget("coordinate", WCoordinateSystem(10));
    return cam;
}

u_int get_order()
{
    int order = -1;
    while (order < 0 || order >= 10)
    {
        cout << "Enter order ]0,11] : ";
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

    Phi phi { order };
    phi.of_zeros();

    for (int n = -int(order); n <= 0; n+=2) 
    {
        cout << "Currently watching : " << order << "-Wavejet for " 
                << "phi(" << order << ", " << n << ") + " 
                << "phi(" << order << ", " << abs(n) << ")" << endl;
        //auto scale = order == 0 ? 1 : order * order;
        auto scale = order == 0 ? 1 : exp(order) / (order * 2);
        phi.wiseset(order, n, complex<double> { 0.1 / (scale), 0.1 / (scale) });

        WavejetDisplay wd { phi, order };
        wd.compute_and_find_point_position();
        wd.display(cam);

        cam.spin();

        string folder = "output/" + to_string(order) + "-Wavejet/";
        string file   = to_string(order) + "_" + to_string(abs(n)) + ".png";

        fs::path path = fs::current_path();
        fs::create_directories(folder);
        bool result = imwrite(path.string() + "/" + folder + file, 
                                cam.getScreenshot(), 
                                vector<int> {IMWRITE_PNG_COMPRESSION, 9});
        if (result)
            printf("Saved PNG file with alpha data.\n");
        else
            printf("ERROR: Can't save PNG file.\n");

        phi.wiseset(order, n, complex<double> { 0, 0 });
        cam = init_window();
    }

    return 0;
}
