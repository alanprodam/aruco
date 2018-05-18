#include "aruco.h"
#include <opencv2/flann.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <vector>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;
using namespace aruco;

int main(int argc, char **argv)
{
    namedWindow("image", CV_WINDOW_NORMAL);

    if (argc != 2)
    {
        std::cerr << "Usage: inimage" << std::endl;
        return -1;
    }
    //read the image
    Mat image = imread(argv[1]);

    //configuration of aruco detector with library MIP_36h12
    MarkerDetector Detector;
    Detector.setDictionary("ARUCO_MIP_36h12");

    //detect
    vector<aruco::Marker> markers = Detector.detect(image);

    //for each marker, draw info and its boundaries in the image
    for (unsigned int i = 0; i < markers.size(); i++)
    {
        // print the corners of markers
        cout << markers[i] << endl;
        markers[i].draw(image, Scalar(0, 0, 255), 12);
    }

    imshow("image", image);
    resizeWindow("image", image.cols / 3, image.rows / 3);

    waitKey(0);
}