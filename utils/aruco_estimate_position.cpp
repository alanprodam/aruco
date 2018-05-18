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

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

int main(int argc, char **argv)
{
    // create the detector o set parameters 
    MarkerDetector Detector;

    // DM_NORMAL - DM_VIDEO_FAST - DM_FAST(the best)
    Detector.setDetectionMode(aruco::DM_FAST);

    // configuration of aruco detector with library MIP_36h12 / second parameter is Minimum Marker Size to increse speed
    Detector.setDictionary("/home/alantavares/aruco-3.0.10/build/utils/myconnect.dict");

    // get camera parameters from xml file
    aruco::CameraParameters camera;
    camera.readFromXMLFile("/home/alantavares/aruco-3.0.6/build/utils/calibration_1920x1080.yml");

    // selecting the best parameters for your problem
    Detector.loadParamsFromFile("arucoConfig.yml");

    // create image frame of video of type Mat
    Mat frame, TheInputImageCopy;

    // set detect the marker size that is 0.326
    float MarkerSize = 0.37f;

    // video capture
    VideoCapture vcap;
    vcap.open(1);
    //vcap.open("/home/alantavares/aruco-3.0.6/build/utils/datasets/dataset_high_resolution_stable.mp4");

    //set height and width of capture frame
    vcap.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    vcap.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    while (vcap.grab())
    {   
        // capture frame
        //vcap.retrieve(frame);
        vcap.read(frame);
        camera.resize(frame.size());

        // detection with frame, parameters of camera e marker size
        auto markers = Detector.detect(frame, camera, MarkerSize);

        // for each marker, draw info and its boundaries in the image
        for (unsigned int i = 0; i < markers.size(); i++)
        {
            //cout << Markers[i].id << endl;
            if (markers[i].id == 231)
            {
                markers[i].draw(frame, Scalar(0, 0, 255), 5);
                CvDrawingUtils::draw3dAxis(frame, markers[i], camera);

                //cout << "rotation " << 57.29f*markers [i].Rvec << endl;
                cout << " translate" << markers[i].Tvec << endl;
                
                //cout << "rotation x: " << cvRound(markers[i].Rvec.ptr<float>(0)[i]) << " y: " << cvRound(markers[i].Rvec.ptr<float>(0)[i]) << " z: " << cvRound(markers[i].Rvec.ptr<float>(0)[i])
                //     << " | translation x: " << cvRound(markers[i].Tvec.ptr<float>(0)[i]) << " y : " << cvRound(markers[i].Tvec.ptr<float>(0)[i]) << " z : " << cvRound(markers[i].Tvec.ptr<float>(0)[i]) << endl;
                putText(frame, "X", Point(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), 1.2, 1.2, Scalar(0, 0, 255), 2);
            }
        }

        // show outputs with frame argumented information
        namedWindow("Video Aruco", CV_WINDOW_NORMAL);
        imshow("Video Aruco", frame);
        resizeWindow("Video Aruco", frame.cols * 1.5, frame.rows * 1.5);

        // print marker info and draw the markers in image
        frame.copyTo(TheInputImageCopy);
        TheInputImageCopy = Detector.getThresholdedImage();

        // show outputs with ThresholdedImage argumented information
        namedWindow("Video thres", CV_WINDOW_NORMAL);
        imshow("Video thres", TheInputImageCopy);
        resizeWindow("Video thres", TheInputImageCopy.cols * 1.5, TheInputImageCopy.rows * 1.5);

        //cout << " size" << frame.cols << "x" << frame.rows << endl;

        waitKey(5);
        //while(char (waitKey(0)) != 27); // wait for esc to be pressed
    }

    return 0;
}