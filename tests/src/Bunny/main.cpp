#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/viz/viz3d.hpp>

#include "wavejet.hpp"
#include "../../tests/randomizer.hpp"


#include <iostream>
#include <filesystem>
#include <fstream>
#include <vector>
#include <algorithm>


// wavejet order demo
constexpr unsigned int ORDER = 2;
constexpr double NEIGHBOURHOOD_RADIUS = 0.5;


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

vector<Point3d> enough_near_points(const vector<Point3d>& pts, 
                                   const Point3d& choosen,
                                   double radius)
{
    vector<cv::Point3d> res;
    std::copy_if(pts.begin(), pts.end(), std::back_inserter(res), 
        [choosen, radius](const Point3d& p) {
        return norm(choosen - p) <= radius && p != choosen;
    });
    return res;
}


int main(int argc, char** argv)
{
    if (argc != 1)
    {
        cout << "wrong args : " << argc << argv << endl;
    }
    auto cam = init_window();

    // BUNNY PART
    ifstream bunny_txt;
    string path = fs::current_path().string() + "/../../resources/cow.txt";
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
        pts.emplace_back(Point3d { x * 1000, y * 1000, z * 1000 });
    }
    cam.showWidget("rabbit", WCloud(pts, Color::black()));

    auto compare_max = [](const Point3d& p1, const Point3d& p2) 
    { 
        return norm(p1) < norm(p2);
    };

    auto farest_point = *max_element(pts.begin(), pts.end(), compare_max);
    vector<double> vec_max { farest_point.x, farest_point.y, farest_point.z };
    auto max = *max_element(vec_max.begin(), vec_max.end());
    double neighbourhood_radius = max / 10;
    cout << "Neighbourhood radius : " << neighbourhood_radius << endl;

    // WAVEJET PART
    u_int cpt = 0;
    for (auto& point : pts)
    {
        //point = pts.at(30);
        cout << "treating ... " << cpt << endl;
        Wavejet wj {ORDER, 
                    point, 
                    enough_near_points(pts, point, neighbourhood_radius), 
                    neighbourhood_radius};
        
        wj.display_svdV(cam);
        wj.display(cam, wj.get_svd(), cpt);
        cpt ++;
        if (cpt >= 3) break; // too long else ...
    }

    cam.spin();

    return 0;
}