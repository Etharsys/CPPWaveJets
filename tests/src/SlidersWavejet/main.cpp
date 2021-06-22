#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/viz/viz3d.hpp>

#include <iostream>
#include <complex>
#include <thread>
#include <chrono>
#include <mutex>
#include <string>
#include <filesystem>


#include "phi.hpp"
#include "wavejetDisplay.hpp"


using namespace cv;
using namespace viz;
using namespace std;

namespace fs = std::filesystem;



Viz3d init_window();
void vizcam();

u_int k = 4;

const double scale_0 = 2;
const double scale_1 = 10;
const double scale_2 = 30;
const double scale_3 = 80;
const double scale_4 = 150;

const int max_scale = 10;

int real_0_0 = 0; int imag_0_0 = 0;
int real_1_1 = 0; int imag_1_1 = 0;
int real_2_2 = 0; int imag_2_2 = 0;
int real_2_0 = 0; int imag_2_0 = 0;
int real_3_3 = 0; int imag_3_3 = 0;
int real_3_1 = 0; int imag_3_1 = 0;
int real_4_4 = 0; int imag_4_4 = 0;
int real_4_2 = 0; int imag_4_2 = 0;
int real_4_0 = 0; int imag_4_0 = 0;


Viz3d cam;

std::mutex lock_track;

Viz3d init_window()
{
    Viz3d cam { "Wavejets Demo" };
    cam.setRepresentation(REPRESENTATION_SURFACE);
    cam.setBackgroundColor(Color::white());
    cam.showWidget("coordinate", WCoordinateSystem(10));
    return cam;
}

void vizcam()
{
    while (!lock_track.try_lock())
    {
        this_thread::sleep_for(chrono::milliseconds(100));
    }
    cam.spin();
}

static Mat wavejet_mat (int k, int n)
{
    Mat img;
    string file_name  = to_string(k) + "_" + to_string(n) + ".png";
    string image_path = samples::findFile(fs::current_path().string() + "/output/" + to_string(k) + "-Wavejet/" + file_name, false, true);
    if (image_path == "")
    {
        cerr << "Cannot found file" << file_name << ", please compute corresponding KWavejet before" << endl;
        return img;
    }
    img = imread(image_path, IMREAD_COLOR);
    if (img.empty())
    {
        cerr << "Error occur while opening image" << endl;
        return img;
    }
    resize(img, img, Size(), 0.1, 0.1);
    putText(img, to_string(k) + "_" + to_string(n), Point(0, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(0,0,0));
    rotate(img, img, ROTATE_90_COUNTERCLOCKWISE);
    return img;
}

static void display_real_wavejets()
{
    Mat wj_add;
    wj_add.push_back(wavejet_mat(0, 0));
    wj_add.push_back(wavejet_mat(1, 1));
    wj_add.push_back(wavejet_mat(2, 2));
    wj_add.push_back(wavejet_mat(2, 0));
    wj_add.push_back(wavejet_mat(3, 3));
    wj_add.push_back(wavejet_mat(3, 1));
    wj_add.push_back(wavejet_mat(4, 4));
    wj_add.push_back(wavejet_mat(4, 2));
    wj_add.push_back(wavejet_mat(4, 0));
    if (!wj_add.empty())
    {
        rotate(wj_add, wj_add, ROTATE_90_CLOCKWISE);
        imshow("Tracks", wj_add);
    }
}

static void on_trackbar( int, void* )
{
    Phi phi { k };
    phi.of_zeros();
    phi.wiseset(0, 0, complex<double> { real_0_0 / scale_0, imag_0_0 / scale_0 });
    phi.wiseset(1, 1, complex<double> { real_1_1 / scale_1, imag_1_1 / scale_1 });
    phi.wiseset(2, 2, complex<double> { real_2_2 / scale_2, imag_2_2 / scale_2 });
    phi.wiseset(2, 0, complex<double> { real_2_0 / scale_2, imag_2_0 / scale_2 });
    phi.wiseset(3, 3, complex<double> { real_3_3 / scale_3, imag_3_3 / scale_3 });
    phi.wiseset(3, 1, complex<double> { real_3_1 / scale_3, imag_3_1 / scale_3 });
    phi.wiseset(4, 4, complex<double> { real_4_4 / scale_4, imag_4_4 / scale_4 });
    phi.wiseset(4, 2, complex<double> { real_4_2 / scale_4, imag_4_2 / scale_4 });
    phi.wiseset(4, 0, complex<double> { real_4_0 / scale_4, imag_4_0 / scale_4 });
    phi.prompt_display();

    display_real_wavejets();

    WavejetDisplay wd { phi, k };
    wd.compute_and_find_point_position();
    wd.display(cam);
    lock_track.unlock();
}


int main(int argc, char** argv)
{
    if (argc != 1)
    {
        cout << "wrong args : " << argc << argv << endl;
    }
    cam = init_window();
    lock_track.lock();

    thread demo_thread { vizcam };

    namedWindow("Tracks", WINDOW_AUTOSIZE); // Create Window
    resizeWindow("Tracks", Size { 1000, 500});
    
    createTrackbar( "Phi(0,0) (real)", "Tracks", &real_0_0, max_scale, on_trackbar );
    createTrackbar( "Phi(0,0) (imag)", "Tracks", &imag_0_0, max_scale, on_trackbar );

    createTrackbar( "Phi(1,1) (real)", "Tracks", &real_1_1, max_scale, on_trackbar );
    createTrackbar( "Phi(1,1) (imag)", "Tracks", &imag_1_1, max_scale, on_trackbar );

    createTrackbar( "Phi(2,2) (real)", "Tracks", &real_2_2, max_scale, on_trackbar );
    createTrackbar( "Phi(2,2) (imag)", "Tracks", &imag_2_2, max_scale, on_trackbar );

    createTrackbar( "Phi(2,0) (real)", "Tracks", &real_2_0, max_scale, on_trackbar );
    createTrackbar( "Phi(2,0) (imag)", "Tracks", &imag_2_0, max_scale, on_trackbar );

    createTrackbar( "Phi(3,3) (real)", "Tracks", &real_3_3, max_scale, on_trackbar );
    createTrackbar( "Phi(3,3) (imag)", "Tracks", &imag_3_3, max_scale, on_trackbar );

    createTrackbar( "Phi(3,1) (real)", "Tracks", &real_3_1, max_scale, on_trackbar );
    createTrackbar( "Phi(3,1) (imag)", "Tracks", &imag_3_1, max_scale, on_trackbar );

    createTrackbar( "Phi(4,4) (real)", "Tracks", &real_4_4, max_scale, on_trackbar );
    createTrackbar( "Phi(4,4) (imag)", "Tracks", &imag_4_4, max_scale, on_trackbar );

    createTrackbar( "Phi(4,2) (real)", "Tracks", &real_4_2, max_scale, on_trackbar );
    createTrackbar( "Phi(4,2) (imag)", "Tracks", &imag_4_2, max_scale, on_trackbar );

    createTrackbar( "Phi(4,0) (real)", "Tracks", &real_4_0, max_scale, on_trackbar );
    createTrackbar( "Phi(4,0) (imag)", "Tracks", &imag_4_0, max_scale, on_trackbar );

    on_trackbar( 0, 0 );

    waitKey(0);

    demo_thread.join();

    return 0;
}