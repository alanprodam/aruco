# aruco-3.0.10
ArUco - 3.0.10

ArUco: One library to rule them all
-------------------------------------------------------------------

ArUco is an easy-to-use C++ library for detection of squared planar markers based on OpenCv. It can detect markers from a variety of ditionaries ARUCO, APRILTAGS, ARTAG and ARTOOLKIT+.
It is a library, to rule them all ;)

CONTACT: Rafael Munoz-Salinas: rmsalinas@uco.es 

For further info read https://www.uco.es/investiga/grupos/ava/node/26

### INTRODUCTION

The library allows to detect squared planar markers in images. A   marker is a squared element with a black border and a inner binary code inside. The set of valid codes is called dictionary. Several libraries have been proposed, and each one usually includes its own dictionary of valid markers. This version of Aruco allows to use the most famous dictionaries: ARUCO,APRILTAGS,ARTAG,CHILITAGS,ARTOOLKIT+, etc.


The library allows to estimate the pose of a camera from one or many  markers.

In any case, we strongly recommend to use our dictionary ARUCO_MIP_36h12,  which in our opinion, is the one that has the best trade-off between size and robustness. See https://www.researchgate.net/publication/282426080_Generation_of_fiducial_marker_dictionaries_using_mixed_integer_linear_programming for more information on dictionary generation.

You can download our dictionary at https://www.uco.es/investiga/grupos/ava/sites/default/files/salinas/aruco_mip_36h12_dict.zip or print it yourself using aruco_print_dictionary util.

The library also allows to use marker maps. A marker map is a set of markers placed in the environment whose location w.r.t. each other does not change. The simplest marker map supported by the library is a set of markers printed in a piece of paper. Localization in  more robust since the misdetection of several markers of the map is not a problem as long as a minimum set of them are detected.

Additionally, marker maps can also extend to much larger environments. Imagine you place markers in a room and you want to locate using them. Now, aruco allows you to do so using the brother library aruco_markermapper.


For additional information to this README file, please visit our web page : http://www.uco.es/investiga/grupos/ava/node/26


### What's new in 3.x and how to port

See Changelog for a more detailed description
		- Removed the use of the parameters for configuring the detection. Now, you only need to use MarkerDetector::setDetectionMode() to specify how fast you want to be.
		- Increased speed. It is now the fastest library for markers
		- Added the capability of detecting markers of any type of markers n case you do not know wich type of dictionary you are using. However, it is prefereable to explicitly indicate the dictionary you are using.¡
		- Calibration board changed
		- Cmake improved


The main improvement of version 3 is related to speed (see below for more info) and the management of dictionaries. We have added the meta dictionary Dicionary::ALL_DICTS. This is now the default dictionary and will detect markers in all the dictionaries.

A great different  with version 2.0 is that you can select amongst a set of Dictionaries:   ARUCO_MIP_25h7,ARUCO_MIP_16h3, and ARUCO_MIP_36h12 which are hardcoded. Thus, you do not need external files describing the ditionary.

We recommend using ARUCO_MIP_36h12, however, for compatibilty with old users, the default dictionary of the library is ARUCO, which is the original dictionary in previuos versions.  However, we recommend using ARUCO_MIP_36h12 because it is more robust to errors.

The concept of Board (In 1.x version) is replaced by  MarkerMaps. 

Also important, from this moment, the axis point in the same direction as given by opencv, i.e., Z axis is perperdicular to the marker. 


With respect to performance, we have improved in several aspects. First, all markers are now analized in parallel (if omp is enabled/found at compile time). Second, we have added a pyramid method for marker detection. Once detected rectangles in the first level, we create the canonical image in the pyramid level that achieves the best trade-off between quality and size. So, the call to wrap function is now more constant. So, you'll not notice decrease in performance as you approach a marker and it becomes bigger.


### COMPILING

Use cmake.

* In linux

(cd path;mkdir build;cd build cmake ..;make -j4;)

In fact, I strongly recommend to install the library into your own working space instead of in the root file system. So I would do:
(cd path;mkdir build;cd build cmake .. -DCMAKE_INSTALL_PREFIX=<pathToInstall>;make -j4 install;)

*  In windows

use cmake-gui, or QtCreator if you have not a lot of experience.

Set the -DOPENCV_DIR variable pointing to the directory where it is.

* To consider

	- DUSE_OWN_EIGEN3=ON/OFF  : We need the eigen3 library. So we have a copy of it in 3drparty. However, if you prefer to use yours, set this parameter to OFF
	- USE_DOUBLE_PRECISION_PNP=ON/OFF  The classes MarkerPoseTracker and   MarkerMapPoseTracker uses an optimization technique to estiamte the pose. By default we use double, but you can disable this option and float will be used instead.

* OpenGL examples



You must set the cmake option -DBUILD_GLSAMPLES=ON in cmake. I.e., cmake .. -DBUILD_GLSAMPLES=ON -DCMAKE_INSTALL_PREFIX=<pathToInstall>

In Ubuntu 16 Install first freeglut3 : sudo apt-get install freeglut3 freeglut3-dev


### TESTING

	Download the aruco test data from sourceforge and run the examples in the utils_xxx folders. Alternatively, windows user have a precompiled version. You have all the programs in the bin folder.
	
### LIBRARY DESCRIPTION:

The ArUco library contents are divided in several  directories. 

src/: contains the library
3rdparty/: contains the external code required. Currently only eigen3 is in it. If you prefer using your own eigen3 see in COMPILING section
utils/: utils for detection of individual markers.
utils_markermap: utils for using markermaps
utils_calibration: camera calibration utils. We provide a chessboard-like calibration board. You do need to see complete to do calibration so it is much better!
utils_gl: examples on how to use the library with OpenGL. Requires to enable -DBUILD_GLSAMPLES=ON in cmake file.



The library main classes are:

   - aruco::Marker: which represent a marker detected in the image
   - aruco::MarkerDetector: that is in charge of deteting the markers in a image Detection is done by simple calling the member funcion ArMarkerDetector::detect(). You can select the dictionary you want to use calling 
		ArMarkerDetector::setDictionary(). (see programs in utils)
   - aruco::MarkerPoseTracker: tracks the pose of the camera wrt to a marker. Instead of recalculating the pose everytime from scratch, the tracker takes advantage of knowing the previous location to refine the current location. This is specially useful when the ambiguity problem arises. The ambiguity problem refers to the fact that some times there are more than one valid poses for the given view. In that cases, the pose returned by the estimator is any of them. However, using the tracker you can avoid this problem in many cases. See utils/aruco_tracker.cpp for an example.

   - aruco::MarkerMap: represents a set of markers whose location is known wrt to each other. (see programs in utils_markermap)
   - aruco::MarkerMapPoseTracker: is the class employed to estimate the pose of the camera wrt the set of markers.
   - aruco::CvDrawingUtils: a class with some routines for drawing in opencv images


### Marker Maps

Marker maps exteds the concept of Board in the previous version of the library

Image you want to locate a robot in a laboratory. Then, you can place markers around the environment, record a video with the camera, and automatically obtain the location of the markers (this is call the marker map).
With the marker map, you can now locate the robot by looking a any of the makers.


You can create simple marker maps with the aruco libary, such as boards. A board is a grid of markers printed on a piece of paper. Since they are all in the same piece of paper, their location wrt each other are known in advance. 

However, for the case of the robot previouly explained, it is not known 3D the locations of the markers. So the  library marker_mapper allows to determine their location from a video sequence, and then, use the result in tacking tasks using aruco library. See the example utils_markermap/aruco_test_markermap.cpp to know how to do the job. The program also generates a pcd file showing the 3d location of the markers and of the camera along its trayectory. Additionally, the program can also output the 3d location to a log file.



### APPLICATIONS

- The library comes with several applications that will help you to learn how to use the library:
 -# utils/aruco_print_marker: which creates marker and saves it in a jpg file you can print.
 -# utils/aruco_print_dictionary: saves to a dictionary all the markers of the dictionary indicated(ARUCO,APRILTAGS,ARTOOLKIT+,etc).
 -# utils/aruco_simple : simple test aplication that detects the markers in an image
 -# utils/aruco_test: this is the main application for detection and profiling. It reads images either from the camera of from a video and detect markers. You have a menu to play with the different parameters of the library. Additionally, if you provide the intrinsics of the camera(obtained by OpenCv calibration) and the size of the marker in meters, the library calculates the marker intrinsics so that you can easily create your AR applications.
 -# utils/aruco_tracker: example showing how to use the tracker. 


 -# utils_markermap/aruco_create_markermap: creation of simple marker maps (the old boards). It creates a grid of markers that can be printed in a piece of paper.
	The result of this program are two files (.png and .yml) The png file is an image of the marker you can print. The .yml file is the configuration file that you'll need to pass to the rest of the programs, so they know how the map is.  The .yml contains the location of the markers in pixels. Since we do not know in advance how large the printed marker will be, we use pixels here. However, in order to obtain the camera, w need to know the real size of the marker. For this pourpose, you can either use the program  utils_markermap/aruco_markermap_pix2meters, that creates a new .yml, with the map information in meters. Alternatively, all the test programs allows you to indicate the markersize in the command line. 
 
 -# utils_markermap/aruco_markermap_pix2meters converts a markermap configuration file from pixels to meters
 -# utils_markermap/aruco_simple_markermap : simple example showing how to determine the camera pose using the marker maps
 -# utils_markermap/aruco_test_markermap : a bit more elaborated example showing how to determine the camera pose using the marker maps with 3D visualization.
 
  
 -# utils_calibration/aruco_calibration : a program to calibrate a camera using a marker set comprised by aruco markers.  It is a marker map,you can download at
  https://www.uco.es/investiga/grupos/ava/sites/default/files/salinas/aruco_calibration_grid_board_a4.pdf

 -# utils_calibration/aruco_calibration_fromimages the same as above, but from images saved in a file

 -# utils_gl/aruco_test_gl simple example showing how to combine aruco with OpenGL
 
 NOTE ON OPENGL: The library supports  the integration with OpenGL. In order to compile with support for OpenGL, you just have  installed in your system the develop packages for GL and glut (or freeglut).
 
### Dictionaries and Speed

### Dictionaries

Aruco version 3 is able to detect from a wide variety of dictionaries. The valid dictionaries are enumerated in Dictionary::DICT_TYPES (dictionary.h). As you can see, we a specific dictionary  named ALL_DICTS. By default, MarkerDetector is a configured in this dictionary. It is not really a dictionary but a metadictionary that represent all the valid ones that the library can detect. So, an user that does not know much about the library can work easily. 

 We strongly recommend to use ARUCO_MIP36h12 (https://www.uco.es/investiga/grupos/ava/sites/default/files/salinas/aruco_mip_36h12_dict.zip) and to explictly indicate it by calling MarkerDetector::setDictionary("ARUCO_MIP36h12")  before starting to call MarkerDetector::detect(). 

If you work using  ALL_DICTS instead of the one you really are using, there are more chances to get detection errors.

Finally, it is important to indicate that Marker::dict_info will tell you the dictionary a marker belong (this is new in version 3).


### Speed

ArUco version 3 main changes are related to speed and to ease of usage. To do so, we define two main concepts:  Minimum Marker Size and Detection Mode.
 
* Minimum Marker Size

In most cases when detecting markers, we are interested in markers of a minumum size. Either we know that markers will have a minimum size in the image, or we are not interested in very small markers because they are not reliable for camera pose estimation. Thus, ArUco 3 defines a method to increase the speed by using images of smaller size for the sake of detection. However, please notice that the precision is not affected by this fact, only computing time is reduced. In order to be general and to adapt to any image size, the minimum marker size is expressed as a normalized value (0,1) indicating the minimum area that a marker must occupy in the image to consider it valid. The value 0 indicates that all markers are considered as valid. As the minimum size increases towards 1, only big markers will be detected.
 
Which value should I use? Well, you should try in your own video sequences to get the best results. However, we have found that setting this value to 0.02 (2% of the image area) has a tremendous impact in the speed. If you are processing video and you want maximum speed, you can use the Detection mode DM_VIDEO_FAST (see below) and it will automatically compute the minimum marker by considering the information in the previous frame.
 
* Detection Mode

(See MarkerDetector::DetectionMode in markerdetector.h) refers to three basic use cases that we can normally find among the ArUco users:
 - DM_NORMAL: this is the case of requiring to detect markers in images, and not taking much care about the computing time. This is normally the case of a batch processing in which  computing time is not a problem. In this case, we apply a local adaptive threshold approach which is very robust. This is the approach used in ArUco version 2.
 - DM_FAST: in this case, you are concern about speed. Then, a global threshold approach is employed, that randomly searches the best threshold. It works fine in most cases.
 - DM_VIDEO_FAST: this is specially designed for processing video sequences. In this mode, a global threshold is automatically determined at each frame, and the minimum marker size is also automatically determined in order achieve the maximum speed. If the marker is seen very big in one image, in the next frame, only markers of similar size are searched.

By default, the MarkerDetector is configured to be conservative, i.e., with minimum marker size to zero, and in DM_NORMAL mode. If you want to change this behaviour, you need to call:

void MarkerDetector::setDetectionMode( DetectionMode dm,float minMarkerSize=0);  

before detecting markers.

 
### Calibration

Camera calibration is a required step if you want to estimate the location of your camera. Aruco has its own calibration board that has an advantage over the classical opencv chessboard: since it is composed by several markers, you do not need to see it completely in order to obtain calibration points. Instead, you can see if partially and still works. This is a very convenient feature in many applications. You have programs in utils_calibration to use calibrate using our chessboard.

To calibrate you should do the following. First, download the calibration board and print it in a piece of paper ( https://www.uco.es/investiga/grupos/ava/sites/default/files/salinas/aruco_calibration_grid_board_a4.pdf)

Use a tape to measure the size of the markers. Please, be very precise. Annotate the size in meters or in your prefered metric. Please notice that when you compute the distance of your camera to the markers, it will be in the metric you use. So, if you measure the size of the markers in meters, then, you camera position will be expressed in meters too.

Once printed and measured, take photos of the board from different locations and perspectives. Take at least 15 photos, and try to see the whole board in all of them.  Then, use the program in utils_calibration/aruco_calibration_fromimages as


aruco_calibration_fromimages mycalibrationfile.yml -size 0.03 image1.jpg image2.jpg ...


the parameter mycalibrationfile.yml will be the output of the process. The parameter 0.03 is the size of each marker (you measured earlier), and then the images used for calibration. Alternatively, you can calibrate a camera connected to your computer using the program aruco_calibration, which is an interactive calibration program.



### Custom Dictionaries

Aruco allows to use your own dictionaries. To do so, first write the definition of your dictionary. Imagine you want to create a dictionary of 3 markers with size 3x3 bits .


You must create then the following file (A copy of this file is in utils/myown.dict )

name MYOWN
nbits  9
010001001
111101010
000001100

Please notice that the bits are expressed from topleft corner to bottomright corner.
Then, you might want to print your dictionary. So, use the aplication utils/aruco_print_dictionary as:
./utilts/aruco_print_dictionary <pathToSaveAllImages>  <pathto/myown.dict>

In the directory indicated as first parameter, the images will be saved. Then, print them and take a picture. To test if it works, use utils/aruco_test. In that case, pass the parameter -d  <pathto/myown.dict>. It will automatically load your dict and use it.


