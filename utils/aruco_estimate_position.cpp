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

void printMenuInfo(Mat &im){
    putText(im, "X", Point(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), 1.2, 1.2, Scalar(0, 0, 255), 2);
}

int main(int argc, char** argv)
{

    try
    {
        // Read camera calibration data from xml file
        TheCameraParameters.readFromXMLFile("/home/alantavares/aruco/utils/filecalibration/camera_result_40_image.yml");

        if (!TheCameraParameters.isValid()) {
            cerr << "Calibration Parameters not valid" << endl;
            return -1;
        }   

        // selecting the best parameters for your problem
        MDetector.loadParamsFromFile("/home/alantavares/aruco/utils/arucoConfig.yml");

        // set detect the marker size that is 0.326
        float MarkerSize = 1.0f;

        TheVideoCapturer.open(1);
        //TheVideoCapturer.open("/home/alantavares/aruco-3.0.6/build/utils/datasets/dataset_high_resolution_stable.mp4");

        // check video is open
        if (TheVideoCapturer.isOpened()){
            //set height and width of capture frame
            TheVideoCapturer.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
            TheVideoCapturer.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
            
            isVideo = true;

        } else{
                throw std::runtime_error("*****Could not open video*****");
        }

        iDictionaryIndex = Dictionary::getTypeFromString( MDetector.getParameters().dictionary );
        cout << "iDictionaryIndex Current = " << Dictionary::getTypeString((aruco::Dictionary::DICT_TYPES) iDictionaryIndex) << endl;
        
        iThreshold = MDetector.getParameters().ThresHold;
        cout << "iThreshold Current = " << iThreshold << endl;
        iCornerMode = MDetector.getParameters().cornerRefinementM;
        cout << "iCornerMode Current = " << iCornerMode << endl;

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
                if (TheMarkers[i].id == 0 || TheMarkers[i].id == 1) //|| TheMarkers[i].id == 2 || TheMarkers[i].id == 3 || TheMarkers[i].id == 4 || TheMarkers[i].id == 5)
                {

                    TheMarkers[i].draw(TheInputImage, Scalar(0, 0, 255),2,true);

                    cout << " LandMarker [" << TheMarkers[i].id << "]: " <<
                        "  Tx: " << TheMarkers[i].Tvec.ptr<float>(0)[0] << " m "<<
                        "\tTy: " << TheMarkers[i].Tvec.ptr<float>(1)[0] << " m "<<
                        "\tTz: " << TheMarkers[i].Tvec.ptr<float>(2)[0] << " m "<<
                        "\tRx: " << TheMarkers[i].Rvec.ptr<float>(0)[0] << " rad "<<
                        "\tRy: " << TheMarkers[i].Rvec.ptr<float>(1)[0] << " rad "<<
                        "\tRz: " << TheMarkers[i].Rvec.ptr<float>(2)[0] << " rad "<< endl;

                    //CvDrawingUtils::draw3dCube(TheInputImage, TheMarkers[i], TheCameraParameters);
                    printMenuInfo(TheInputImage);
                    CvDrawingUtils::draw3dAxis(TheInputImage, TheMarkers[i], TheCameraParameters);
                }


            }

            // show outputs with frame argumented information
            namedWindow("Video Aruco", CV_WINDOW_NORMAL);
            imshow("Video Aruco", TheInputImage);
            resizeWindow("Video Aruco", 1020,720);
            //resizeWindow("Video Aruco", TheInputImage.cols * 1.5, TheInputImage.rows * 1.5);
            //moveWindow("Video Aruco", 2000, 100);

            // print marker info and draw the markers in image
            TheInputImage.copyTo(TheInputImageCopy);
            TheInputImageCopy = MDetector.getThresholdedImage();

            // show outputs with ThresholdedImage argumented information
            namedWindow("Video thres", CV_WINDOW_NORMAL);
            imshow("Video thres", TheInputImageCopy);
            resizeWindow("Video thres", 1020,720);
            //resizeWindow("Video thres", TheInputImageCopy.cols * 1.5, TheInputImageCopy.rows * 1.5);
            //moveWindow("Video Aruco", 2000+100, 100);

            //cout << " size" << frame.cols << "x" << frame.rows << endl;
            waitKey(1);         
        }
    }catch(std::exception& ex){
        cout << "Exception :" << ex.what() << endl;
    }
    return 0;
}