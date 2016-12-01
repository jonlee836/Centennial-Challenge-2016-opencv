#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <time.h> 
#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>
#include <string>
#include <sstream>
#include <chrono>
#include <cstdlib>
#include <stdlib.h>
#include <vector>

#include "RoadDetection.h"

using namespace cv;
using namespace std;

Point RoadDetection::FindRoad(Mat& Input){
  
  Point road_XY = Point(-1,-1);
  Rect bounding_rect;
  Scalar color = Scalar(0,255,0);

  vector<Mat> channels;
  vector<Vec4i> hierarchy;
  vector< vector<Point> > Contours1;

  Mat inputCopy = Input.clone();
  Mat erodeElement = getStructuringElement(MORPH_RECT,Size(20,20));
  Mat dilateElement = getStructuringElement(MORPH_RECT,Size(5,5));

  cvtColor(Input, Input, CV_BGR2Luv);
  split(Input, channels);
  Input = channels[2].clone();
  threshold(Input, Input, 150, 255, 1);
  
  erode(Input, Input, erodeElement);
  dilate(Input, Input, dilateElement);
  findContours(Input, Contours1, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  
  int largest_area = 0;
  int largest_contour_index;

  for( int i = 0; i< Contours1.size(); i++ ){
    double a=contourArea(Contours1[i],false);
    if(a>largest_area){
      largest_area=a;
      largest_contour_index=i;               
      bounding_rect=boundingRect(Contours1[i]);
    }
  }

  for(int i = 0; i < Contours1.size(); i++){
    drawContours(inputCopy, Contours1, i, color, 2, 8, hierarchy, 1, Point());
  }
  
  int oldX = bounding_rect.br().x - (bounding_rect.width/2);
  int oldY = bounding_rect.br().y - (bounding_rect.height/2);

  bounding_rect.width  = 100;
  bounding_rect.height = 100;

  int nX = bounding_rect.br().x - (bounding_rect.width/2);  
  int nY = bounding_rect.br().y - (bounding_rect.height/2); 

  int shiftX = nX - oldX;
  int shiftY = nY - oldY;
       
  bounding_rect.x -= shiftX; 
  bounding_rect.y -= shiftY;

  road_XY.x = bounding_rect.br().x - bounding_rect.width/2;
  road_XY.y = bounding_rect.br().y - bounding_rect.height/2;

  rectangle(inputCopy, bounding_rect.tl(), bounding_rect.br(), Scalar(0,0,255), 1, 8, 0);

  // imshow("contours", inputCopy);

  cout << road_XY << endl;

  return road_XY;

}
