#include "opencv_example.h"

using namespace std;
#include <iostream>
#include <string>
#include <opencv2/highgui.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/core/mat.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

std::string imagepath = "~/Documents/TU Delft master/MicroAirVehicles/AE4317_2019_datasets/cyberzoo_poles/20190121-135009/82978075.jpg"; 
Mat figure = imread(imagepath, IMREAD_COLOR);
cvtColor(figure, figure, CV_BGR2YUV_Y422);

