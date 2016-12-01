#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <time.h> 
#include <cmath>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <stdlib.h>

using namespace cv;
using namespace std;

class RoadDetection
{
  
 private :
  
 public :

  RoadDetection() {
          
  }

  Point FindRoad(Mat& Input);

};
 
#endif
