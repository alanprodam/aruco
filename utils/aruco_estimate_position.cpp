#include "cvdrawingutils.h"
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>
#include <stdexcept>

using namespace std;
using namespace cv;
using namespace aruco;

// default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;


// create the detector o set parameters 
MarkerDetector MDetector;
// video capture
VideoCapture TheVideoCapturer;
// Create the vector marker to set the parameters of The Markers
vector<Marker> TheMarkers;
// create image frame of video of type Mat
Mat TheInputImage, TheInputImageCopy;
// create the camera parameters to set ArUrco
CameraParameters TheCameraParameters;


// parameters of ArUrco MarkerSize
int iDetectMode = 0,iMinMarkerSize = 0;
int iCornerMode;
int iThreshold;
int iCorrectionRate = 0;
int iDictionaryIndex = 0;

bool isVideo=false;

void setParamsFromGlobalVariables(MarkerDetector &md){


    md.setDetectionMode((DetectionMode)iDetectMode,float(iMinMarkerSize)/1000.);
    md.getParameters().setCornerRefinementMethod((CornerRefinementMethod) iCornerMode);

    md.getParameters().ThresHold = iThreshold;
    if(Dictionary::getTypeFromString( md.getParameters().dictionary) != Dictionary::CUSTOM){
        md.setDictionary((Dictionary::DICT_TYPES) iDictionaryIndex,float(iCorrectionRate)/10. );  // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
    }
}

void printMenuInfo(){
        
        cout << "Dictionary= " << aruco::Dictionary::getTypeString((aruco::Dictionary::DICT_TYPES) iDictionaryIndex) << endl;

        // cv::putText(image,str,cv::Size(10,20),FONT_HERSHEY_SIMPLEX, 0.35,cv::Scalar(0,0,0),1);

        // str="Detection Mode="+MarkerDetector::Params::toString(MDetector.getParameters().detectMode);
        // cv::putText(image,str,cv::Size(10,40),FONT_HERSHEY_SIMPLEX, 0.35,cv::Scalar(0,0,0),1);
        // str="Corner Mode="+MarkerDetector::Params::toString(MDetector.getParameters().cornerRefinementM);;
        // cv::putText(image,str,cv::Size(10,60),FONT_HERSHEY_SIMPLEX, 0.35,cv::Scalar(0,0,0),1);
        // cv::imshow("menu",image);
}

int main(int argc, char** argv)
{

    try
    {
        
        //setParamsFromGlobalVariables(MDetector);

        // DM_NORMAL - DM_VIDEO_FAST - DM_FAST(the best)
        //MDetector.setDetectionMode(DM_FAST);
        //MDetector.getParameters().setCornerRefinementMethod((CornerRefinementMethod) iCornerMode);
        //MDetector.getParameters().ThresHold = iThreshold;

        // configuration of aruco detector with library MIP_36h12 / second parameter is Minimum Marker Size to increse speed
        //MDetector.setDictionary("/home/alantavares/aruco/utils/myconnect.dict");

        // Read camera calibration data from xml file
        TheCameraParameters.readFromXMLFile("/home/alantavares/aruco/utils/filecalibration/camera_result_4_image.yml");

        if (!TheCameraParameters.isValid()) {
            cerr << "Calibration Parameters not valid" << endl;
            return -1;
        }   

        // selecting the best parameters for your problem
        MDetector.loadParamsFromFile("/home/alantavares/aruco/utils/arucoConfig.yml");

        // set detect the marker size that is 0.326
        float MarkerSize = 0.095f;

        TheVideoCapturer.open(1);
        //TheVideoCapturer.open("/home/alantavares/aruco-3.0.6/build/utils/datasets/dataset_high_resolution_stable.mp4");

        // check video is open
        if (TheVideoCapturer.isOpened()){
            //set height and width of capture frame
            TheVideoCapturer.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
            TheVideoCapturer.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
        } else{
                throw std::runtime_error("*****Could not open video*****");
        }

        cout << "Dictionary= " << aruco::Dictionary::getTypeString((aruco::Dictionary::DICT_TYPES) iDictionaryIndex) << endl;
        
        while (true)
        {          
            // capture frame
            //TheVideoCapturer.retrieve(frame);

            // read first image to get the dimensions
            TheVideoCapturer >> TheInputImage;
            if (TheCameraParameters.isValid()){
                TheCameraParameters.resize(TheInputImage.size());
            } 

            // detection with frame, parameters of camera e marker size
            TheMarkers = MDetector.detect(TheInputImage, TheCameraParameters, MarkerSize);

            // for each marker, draw info and its boundaries in the image
            for (unsigned int i = 0; i < TheMarkers.size(); i++)
            {
                //if (TheMarkers[i].id == 0)
                {
                    CvDrawingUtils::draw3dAxis(TheInputImage, TheMarkers[i], TheCameraParameters);

                    TheMarkers[i].draw(TheInputImage, Scalar(0, 0, 255),2,true);

                    // cout << " Translate [" << TheMarkers[i].id << "]: " << 
                    //     "  x: " << TheMarkers[i].Tvec.ptr<float>(0)[i] <<
                    //     "\ty: " << TheMarkers[i].Tvec.ptr<float>(1)[i] <<
                    //     "\tz: " << TheMarkers[i].Tvec.ptr<float>(2)[i] << endl;
                
                    cout << " Translate [" << TheMarkers[i].id << "]: " << TheMarkers[i].Tvec << endl;

                    // CvDrawingUtils::draw3dCube(TheInputImage, TheMarkers[i], TheCameraParameters);
                    putText(TheInputImage, "X", Point(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), 1.2, 1.2, Scalar(0, 0, 255), 2);
                }
            }

            // show outputs with frame argumented information
            namedWindow("Video Aruco", CV_WINDOW_NORMAL);
            imshow("Video Aruco", TheInputImage);
            //moveWindow("Video Aruco", 2000, 100);
            resizeWindow("Video Aruco", TheInputImage.cols * 1, TheInputImage.rows * 1);

            // print marker info and draw the markers in image
            TheInputImage.copyTo(TheInputImageCopy);
            TheInputImageCopy = MDetector.getThresholdedImage();

            // show outputs with ThresholdedImage argumented information
            namedWindow("Video thres", CV_WINDOW_NORMAL);
            imshow("Video thres", TheInputImageCopy);
            resizeWindow("Video thres", TheInputImageCopy.cols * 1.5, TheInputImageCopy.rows * 1.5);

            //cout << " size" << frame.cols << "x" << frame.rows << endl;
            waitKey(100);         
        }
    }catch(std::exception& ex){
        cout << "Exception :" << ex.what() << endl;
    }
    return 0;
}