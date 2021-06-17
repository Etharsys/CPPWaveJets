#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/viz/viz3d.hpp>

#include <iostream>
#include <complex>
#include <thread>
#include <chrono>
#include <mutex>



#include "phi.hpp"
#include "wavejetDisplay.hpp"


using namespace cv;
using namespace viz;
using namespace std;


Viz3d init_window();
void vizcam();


const double scale_0 = 2;
const double scale_1 = 10;
const double scale_2 = 30;
const double scale_3 = 80;

const int max_0_0 = 10;
const int max_1_1 = 10;
const int max_2_2 = 10;
const int max_2_0 = 10;
const int max_3_3 = 10;
const int max_3_1 = 10;


int real_0_0 = 0; int imag_0_0 = 0;
int real_1_1 = 0; int imag_1_1 = 0;
int real_2_2 = 0; int imag_2_2 = 0;
int real_2_0 = 0; int imag_2_0 = 0;
int real_3_3 = 0; int imag_3_3 = 0;
int real_3_1 = 0; int imag_3_1 = 0;

Viz3d cam;

std::mutex lock_track;

Viz3d init_window()
{
    Viz3d cam { "Wavejets Demo" };
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

static void on_trackbar( int, void* )
{
    u_int k = 3;

    Phi phi { k };
    phi.of_zeros();
    phi.wiseset(0, 0, complex<double> { real_0_0 / scale_0, imag_0_0 / scale_0 });
    phi.wiseset(1, 1, complex<double> { real_1_1 / scale_1, imag_1_1 / scale_1 });
    phi.wiseset(2, 2, complex<double> { real_2_2 / scale_2, imag_2_2 / scale_2 });
    phi.wiseset(2, 0, complex<double> { real_2_0 / scale_2, imag_2_0 / scale_2 });
    phi.wiseset(3, 3, complex<double> { real_3_3 / scale_3, imag_3_3 / scale_3 });
    phi.wiseset(3, 1, complex<double> { real_3_1 / scale_3, imag_3_1 / scale_3 });
    phi.prompt_display();

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
    
    createTrackbar( "Phi(0,0) (real)", "Tracks", &real_0_0, max_0_0, on_trackbar );
    createTrackbar( "Phi(0,0) (imag)", "Tracks", &imag_0_0, max_0_0, on_trackbar );

    createTrackbar( "Phi(1,1) (real)", "Tracks", &real_1_1, max_1_1, on_trackbar );
    createTrackbar( "Phi(1,1) (imag)", "Tracks", &imag_1_1, max_1_1, on_trackbar );

    createTrackbar( "Phi(2,2) (real)", "Tracks", &real_2_2, max_2_2, on_trackbar );
    createTrackbar( "Phi(2,2) (imag)", "Tracks", &imag_2_2, max_2_2, on_trackbar );

    createTrackbar( "Phi(2,0) (real)", "Tracks", &real_2_0, max_2_0, on_trackbar );
    createTrackbar( "Phi(2,0) (imag)", "Tracks", &imag_2_0, max_2_0, on_trackbar );

    createTrackbar( "Phi(3,3) (real)", "Tracks", &real_3_3, max_3_3, on_trackbar );
    createTrackbar( "Phi(3,3) (imag)", "Tracks", &imag_3_3, max_3_3, on_trackbar );

    createTrackbar( "Phi(3,1) (real)", "Tracks", &real_3_1, max_3_1, on_trackbar );
    createTrackbar( "Phi(3,1) (imag)", "Tracks", &imag_3_1, max_3_1, on_trackbar );

    on_trackbar( 0, 0 );

    waitKey(0);

    demo_thread.join();

    return 0;
}