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


using namespace cv;
using namespace viz;
using namespace std;


namespace fs = std::filesystem;

constexpr int MIN_POINTS = 15;
constexpr double RADIUS_STEP = 1.1;



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
        [choosen, radius](const Point3d& p) 
    {
        return norm(choosen - p) <= radius && p != choosen;
    });
    return res;
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
    auto cam = init_window();

    // BUNNY PART
    ifstream surface_txt;
    string path = fs::current_path().string() + "/../../resources/cow.txt";
    surface_txt.open(path);
    if (!surface_txt.is_open())
    {
        cerr << "Cannot open bunny.txt with path : " << path << endl;
        return 1;
    }
    string line;
    vector<Point3d> pts;

    while(getline(surface_txt, line))
    {
        double x = stod(line.substr(0, line.find(' ')));
        line = line.substr(line.find(' ') + 1);
        double y = stod(line.substr(0, line.find(' ')));
        line = line.substr(line.find(' ') + 1);
        double z = stod(line.substr(0, line.find(' ')));
        pts.emplace_back(Point3d { x, y, z });
    }
    cam.showWidget("cloud", WCloud(pts, Color::black()));
    //cout << pts << endl;

    // WAVEJET PART
    u_int cpt = 0;
    for (auto& point : pts)
    {
        //point = pts.at(14);
        cout << "treating ... " << cpt << endl;
        double neighbourhood_radius = 0.01;
        vector<Point3d> neighbors {};
        while (neighbors.size() <= MIN_POINTS) 
        {
            neighbors = enough_near_points(pts, point, neighbourhood_radius);
            neighbourhood_radius *= RADIUS_STEP;
            //cout << neighbors.size() << endl;
        }

        Wavejet wj {u_int(order), point, neighbors, neighbourhood_radius };
        
        wj.display(cam, wj.get_svd(), cpt);
        cpt ++;
        if (cpt >= 100) break; // too long else ...
    }

    cam.spin();

    return 0;
}