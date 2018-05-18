#include "aruco.h"
#include <opencv2/flann.hpp>
#include <opencv2/core.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <vector>
#include <iostream>
#include <cmath>
#include <sstream>
#include <string>

using namespace std;
using namespace cv;
using namespace aruco;

int main(int argc, char **argv)
{
    // configuration of aruco detector with library MIP_36h12
    MarkerDetector Detector;
    Detector.setDetectionMode(aruco::DM_FAST);
    Detector.setDictionary("ARUCO_MIP_36h12");
    Mat frame;

    // video capture
    VideoCapture vcap;
    vcap.open(argv[1]);

    while (vcap.grab())
    //while(1)
    {   
        // capture frame
        //vcap.retrieve(frame);
        vcap.read(frame);

        // detect
        vector<aruco::Marker> Markers = Detector.detect(frame);

        // for each marker, draw info and its boundaries in the image
        for (unsigned int i = 0; i < Markers.size(); i++)
        {
            // print the corners of markers
            //cout << Markers[i].id << endl;
            if (Markers[i].id == 231){
                Markers[i].draw(frame, Scalar(0, 0, 255), 5);
            } 
        }

        // show inputs with argumented information
        namedWindow("Video Aruco", CV_WINDOW_FREERATIO);
        imshow("Video Aruco", frame);
        resizeWindow("Video Aruco", frame.cols / 2, frame.rows / 2);

        waitKey(20);
        //while(char (waitKey(0)) != 27); // wait for esc to be pressed
    }

    return 0;
}