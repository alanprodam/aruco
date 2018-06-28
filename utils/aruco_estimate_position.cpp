#include "cvdrawingutils.h"
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>
#include <stdexcept>
#include "/home/alantavares/aruco/mavlink/common/mavlink.h"

using namespace std;
using namespace cv;
using namespace aruco;

// default capture width and height
const int FRAME_WIDTH = 1280; //640;
const int FRAME_HEIGHT = 720; //480;
const int inputfps = 30;

// create the detector o set parameters 
MarkerDetector MDetector;
// video capture
VideoCapture TheVideoCapturer;
// Create the vector marker to set the parameters of The Markers
vector<Marker> TheMarkers;
// create image frame of video of type Mat
Mat TheInputImage, TheInputImageCopy, TheInputImageThCopy;
// create the camera parameters to set ArUrco
CameraParameters TheCameraParameters;

// use a map so that for each id, we use a different pose tracker
std::map<uint32_t, MarkerPoseTracker> MTracker; 

// parameters of ArUrco MarkerSize
int iDetectMode = 1,iMinMarkerSize = 0;
int iCornerMode = 0;
int iThreshold;
int iCorrectionRate;
int iDictionaryIndex;
int iEnclosed = 0;
int iShowAllCandidates = 0;

bool isVideo=false;

struct TimerAvrg
{
    std::vector<double> times;
    size_t curr = 0, n;
    std::chrono::high_resolution_clock::time_point begin, end;
    TimerAvrg(int _n = 30)
    {
        n = _n;
        times.reserve(n);
    }
    inline void start() { begin = std::chrono::high_resolution_clock::now(); }
    inline void stop()
    {
        end = std::chrono::high_resolution_clock::now();
        double duration = double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) * 1e-6;
        if (times.size() < n)
            times.push_back(duration);
        else
        {
            times[curr] = duration;
            curr++;
            if (curr >= times.size())
                curr = 0;
        }
    }
    double getAvrg()
    {
        double sum = 0;
        for (auto t : times)
            sum += t;
        return sum / double(times.size());
    }
};

TimerAvrg Fps;

// MAVLink library interface
int sysid = 200;
int compid = 0;
int type = MAV_TYPE_QUADROTOR;

uint8_t system_type = MAV_TYPE_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
uint8_t system_mode = 209;
uint32_t custom_mode = 2;
uint8_t system_state = MAV_STATE_STANDBY;

void setParamsFromGlobalVariables(aruco::MarkerDetector &md)
{
    cout << "\n\n*************Parameters*************" << endl;

    iDictionaryIndex = Dictionary::getTypeFromString(md.getParameters().dictionary);
    if (aruco::Dictionary::getTypeFromString(md.getParameters().dictionary) != Dictionary::CUSTOM) {
        md.setDictionary((aruco::Dictionary::DICT_TYPES)iDictionaryIndex, float(iCorrectionRate) / 10.); // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
    }
    cout << "Dictionary = " << Dictionary::getTypeString((aruco::Dictionary::DICT_TYPES)iDictionaryIndex) << endl;

    iDetectMode = md.getParameters().detectMode;
    md.setDetectionMode((DetectionMode)iDetectMode, float(iMinMarkerSize) / 1000.);
    cout << "DetectMode Mode = " << MarkerDetector::Params::toString((DetectionMode)iDetectMode) << endl;

    iThreshold = md.getParameters().ThresHold;
    md.getParameters().ThresHold = iThreshold;
    cout << "Threshold Current = " << iThreshold << endl;

    md.getParameters().setCornerRefinementMethod((aruco::CornerRefinementMethod)iCornerMode);
    iCornerMode = md.getParameters().cornerRefinementM;
    cout << "CornerMode Mode = " << MarkerDetector::Params::toString(md.getParameters().cornerRefinementM) << endl;

    md.getParameters().detectEnclosedMarkers(iEnclosed);
    if (iEnclosed == 1){
        cout << "Enclosed Mode = Enclosed" << endl;
    } else{
        cout << "Enclosed Mode = No Enclosed" << endl;
    } 

    uint32_t marker_history = 30;
    cout << "MarkerHistory = " << marker_history << endl;    
}

void printMenuInfo(Mat &im, int i)
{
    float fs = float(im.cols) / float(1000);
    putText(im, "fps = " + to_string(1. / Fps.getAvrg()), Point(10, fs * 30), 1.5, 1.5, Scalar(255, 150, 0), 2.2);
    putText(im, "Size Image: " + to_string(im.cols) + "x" + to_string(im.rows), Point(10, fs * 60), 1.5, 1.5, Scalar(255, 150, 0), 2.2);
    putText(im, "Tz: " + to_string(TheMarkers[i].Tvec.ptr<float>(2)[0]), Point(10, fs * 90), 1.5, 1.5, Scalar(255, 150, 0), 2.2);
    putText(im, "X", Point(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), 1.2, 1.2, Scalar(0, 0, 255), 2);
}

int main(int argc, char** argv)
{
    try
    {
        // Read camera calibration data from xml file
        //TheCameraParameters.readFromXMLFile("/home/alantavares/aruco/utils/filecalibration/calibration_smartphone_640x480.yml");
        TheCameraParameters.readFromXMLFile("/home/alantavares/aruco/utils/filecalibration/calibration_smartphone_1280x720.yml");

        if (!TheCameraParameters.isValid()) {
            cerr << "Calibration Parameters not valid" << endl;
            return -1;
        }   

        // selecting the best parameters for your problem
        MDetector.loadParamsFromFile("/home/alantavares/aruco/utils/arucoConfig.yml");

        // set detect the marker size that is 0.326
        float MarkerSize = 1.0f;

        //TheVideoCapturer.open(1);
        //TheVideoCapturer.open("/home/alantavares/Datasets/Novo-Marcador/teste1-marcador-03-640-480.mp4");
        TheVideoCapturer.open("/home/alantavares/Datasets/Novo-Marcador/teste2-marcador-03-1280-720.mp4");

        // check video is open
        if (TheVideoCapturer.isOpened()){
            //set height and width of capture frame
            TheVideoCapturer.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
            TheVideoCapturer.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
            //TheVideoCapturer.set(CV_CAP_PROP_FPS, inputfps);
            //cout << "FPS Current = " << CV_CAP_PROP_FPS << endl;

            isVideo = true;

        } else{
                throw std::runtime_error("*****Could not open video*****");
        }

        setParamsFromGlobalVariables(MDetector);

        int makerHistory1 = 0;
        int makerHistory3 = 0;
        // 1014 histórico de registro do marcador (NORMAL)
        // 1009 histórico de registro do marcador (FAST)

        while (true)
        {          
            // capture frame
            //TheVideoCapturer.retrieve(TheInputImage);

            // read first image to get the dimensions
            TheVideoCapturer >> TheInputImage;
            if (TheCameraParameters.isValid()){
                TheCameraParameters.resize(TheInputImage.size());
            }

            Fps.start();
            // detection with frame, parameters of camera e marker size
            //TheMarkers = MDetector.detect(TheInputImage, TheCameraParameters, MarkerSize);
            TheMarkers = MDetector.detect(TheInputImage);
            Fps.stop();

            // chekc the speed by calculating the mean speed of all iterations
            //cout << "\rTime detection = " << Fps.getAvrg()*1000 << " milliseconds nmarkers = " << TheMarkers.size() << std::endl;

            // print marker info and draw the markers in image
            TheInputImage.copyTo(TheInputImageCopy);

            if (iShowAllCandidates == 1)
            {
                auto candidates = MDetector.getCandidates();
                for (auto cand : candidates)
                    Marker(cand, -1).draw(TheInputImageCopy, Scalar(255, 0, 255));
            }



            
            // for each marker, draw info and its boundaries in the image
            for (unsigned int i = 0; i < TheMarkers.size(); i++)
            {
                if (TheMarkers[i].id == 1) //|| TheMarkers[i].id == 1 || TheMarkers[i].id == 2 || TheMarkers[i].id == 3 || TheMarkers[i].id == 4 || TheMarkers[i].id == 5)
                {
                    MTracker[TheMarkers[i].id].estimatePose(TheMarkers[i], TheCameraParameters, MarkerSize);
                }

                if (TheMarkers[i].id == 3) //|| TheMarkers[i].id == 1 || TheMarkers[i].id == 2 || TheMarkers[i].id == 3 || TheMarkers[i].id == 4 || TheMarkers[i].id == 5)
                {
                    MTracker[TheMarkers[i].id].estimatePose(TheMarkers[i], TheCameraParameters, 0.095);
                }
            }




            // for each marker, draw info and its boundaries in the image
            for (unsigned int i = 0; i < TheMarkers.size(); i++)
            {
                if (TheMarkers[i].id == 1)
                {
                    // green
                    //TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 255, 0, 0), 2, CV_AA);

                    // red maker
                    TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255, 0), 2, CV_AA);

                    makerHistory1++;
                    cout << " makerHistory[" << TheMarkers[i].id << "]: " << makerHistory1 << endl;
                    // translatio and rotation
                    cout << " LandMarker [" << TheMarkers[i].id << "]: " << MTracker[TheMarkers[i].id].getTvec() << endl;
                }

                if (TheMarkers[i].id == 3) //|| TheMarkers[i].id == 1 || TheMarkers[i].id == 2 || TheMarkers[i].id == 3 || TheMarkers[i].id == 4 || TheMarkers[i].id == 5)
                {
                    // green
                    //TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 255, 0, 0), 2, CV_AA);

                    // red maker
                    TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255, 0), 2, CV_AA);

                    makerHistory3++;
                    cout << " makerHistory[" << TheMarkers[i].id << "]: " << makerHistory3 << endl;
                    // translatio and rotation
                    cout << " LandMarker [" << TheMarkers[i].id << "]: " << MTracker[TheMarkers[i].id].getTvec() << endl;
                }
            }



            // // draw a 3d cube in each marker if there is 3d info
            // if (CamParam.isValid())
            // {
            //     for (unsigned int i = 0; i < Markers.size(); i++)
            //     {
            //         if (Markers[i].isPoseValid())
            //         {
            //             //CvDrawingUtils::draw3dCube(InImage, Markers[i], CamParam);
            //             //CvDrawingUtils::draw3dAxis(InImage, Markers[i], CamParam);
            //         }
            //     }
            // }

            // show outputs with frame argumented information
            namedWindow("Video Aruco", CV_WINDOW_NORMAL);
            imshow("Video Aruco", TheInputImageCopy);
            resizeWindow("Video Aruco", 1020,720);
            //resizeWindow("Video Aruco", TheInputImageCopy.cols * 1.5, TheInputImageCopy.rows * 1.5);
            //moveWindow("Video Aruco", 2000, 100);

            // print marker info and draw the markers in image
            TheInputImageCopy.copyTo(TheInputImageThCopy);
            TheInputImageThCopy = MDetector.getThresholdedImage();

            // show outputs with ThresholdedImage argumented information
            namedWindow("Video thres", CV_WINDOW_NORMAL);
            imshow("Video thres", TheInputImageThCopy);
            resizeWindow("Video thres", 1020,720);
            //resizeWindow("Video thres", TheInputImageThCopy.cols * 1.5, TheInputImageThCopy.rows * 1.5);
            //moveWindow("Video Aruco", 2000+100, 100);

            //cout << " size" << frame.cols << "x" << frame.rows << endl;
            waitKey(1);         
        }
    }catch(std::exception& ex){
        cout << "Exception :" << ex.what() << endl;
    }
    return 0;
}                    