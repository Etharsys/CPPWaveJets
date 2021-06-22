#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/viz/viz3d.hpp>

#include <iostream>
#include <filesystem>
#include <fstream>
#include <vector>



using namespace cv;
using namespace viz;
using namespace std;


namespace fs = std::filesystem;



Viz3d init_window()
{
    Viz3d cam { "Wavejets Demo" };
    cam.setRepresentation(REPRESENTATION_SURFACE);
    cam.setBackgroundColor(Color::white());
    //cam.showWidget("coordinate", WCoordinateSystem(10));
    return cam;
}


int main(int argc, char** argv)
{
    if (argc != 1)
    {
        cout << "wrong args : " << argc << argv << endl;
    }
    auto cam = init_window();

    ifstream bunny_txt;
    string path = fs::current_path().string() + "/../../resources/bunny_dots.txt";
    bunny_txt.open(path);
    if (!bunny_txt.is_open())
    {
        cerr << "Cannot open bunny.txt with path : " << path << endl;
        return 1;
    }
    string line;
    vector<Point3d> pts;

    while(getline(bunny_txt, line))
    {
        double x = stod(line.substr(0, line.find(' ')));
        line = line.substr(line.find(' ') + 1);
        double y = stod(line.substr(0, line.find(' ')));
        line = line.substr(line.find(' ') + 1);
        double z = stod(line.substr(0, line.find(' ')));
        pts.emplace_back(Point3d { x, y, z });
    }
    cam.showWidget("rabbit", WCloud(pts, Color::black()));

    cam.spin();

    return 0;
}