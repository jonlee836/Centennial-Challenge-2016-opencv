#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cmath>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>

using namespace cv;
using namespace std;

const int ADDWEIGHT_WHITES = 253;
const int ADDWEIGHT_BLUES = 250;
const int ADDWEIGHT_REDS  = 115;

const int WHITE_MIN_AREA = 300;
const int WHITE_ERODE_AREA = 5;
const int BLUE_MIN_AREA = 300;
const int BLUE_ERODE_AREA = 3;

class ObjectDetection
{
  
 private :

  bool HitsConfirmed = false;
  
  int cl = 5;

  int minArea, whiteMinArea, whiteErodeArea, blueMinArea, blueErodeArea;
  int addweight_whites, addweight_blues, addweight_reds;

  int dc = 0;
  int tc = 0;

  int hitCount        = 0;
  int hitCountlimit   = 9;

  int ShapeLimit      = 125;

  int maxDist = 100;
  
  int wp_count = 0;
  int bp_count = 0;
  int rp_count = 0;

  int nonsenseLine = 350;

  Point Robot_XY     = Point(-1,-1);

  Point curr_wp      = Point(-1,-1);
  Point curr_bp      = Point(-1,-1);
  Point curr_rp      = Point(-1,-1);

  Point prev_wp      = Point(-1,-1);
  Point prev_bp      = Point(-1,-1);
  Point prev_rp      = Point(-1,-1);
  
  Point prevPoint    = Point(-1,-1);
  Point currPoint    = Point(-1,-1);

  Mat Objfound;

  bool HAILHYDRA = false;
  bool currStr = false;

  Scalar colorBlack = Scalar(0);
  Scalar colorGreen = Scalar(0,255,0);

 public :

  ObjectDetection() {
	
    addweight_whites = ADDWEIGHT_WHITES;
    addweight_blues  = ADDWEIGHT_BLUES;
    addweight_reds   = ADDWEIGHT_REDS;

    whiteMinArea = WHITE_MIN_AREA;
    whiteErodeArea = WHITE_ERODE_AREA;
    blueMinArea = BLUE_MIN_AREA;
    blueErodeArea = BLUE_ERODE_AREA;
  }
  void setVariables(
		    int addweight_whites_, int addweight_blues_, int addweight_reds_,
		    int whiteMinArea_, int whiteErodeMin_, 
		    int blueMinArea_, int blueErodeArea_
		    );
  
  Mat FindStuff(Mat& Input, bool showStitch=true);

  void Perform_ImgAdjust(Mat& getWhite, Mat& getBlues, Mat& getReds);

  Mat Perform_GetBlobWhite(Mat& foo, int addweight);
  Mat Perform_GetBlobBlues(Mat& foo, int addweight);
  Mat Perform_GetBlobReds (Mat& foo, int addweight);

  void Perform_UI(Mat& Objfound);
  void Perform_ErodeDilate(Mat& Blobs, int minErodeArea, int minColorArea, Scalar ObjColor);
  void ApplyMarker(Point& drawPoint, string& ObjName, Scalar color, vector <int>& FinalArea, Mat& Objfound);
  void ApplyCrosshair(Point& drawPoint, Mat& Objfound);
  void adjustImg(Mat& Input);
  void ApplyString(Mat& Objfound, Point& currPoint);
  void niceText(Mat& Objfound, string str, Point textStart, Scalar color);
  void RemoveBySize(Mat &a, int minArea);

  // *****************************************************************************

  // step 3
  Point CheckCurrPrev(Point& a, Point& b, int& count, int& countlimit);

  // step 2
  Point CheckOneOfThree(Point& a, Point& b, int& count, int& countlimit);
  Point CheckThreePoints(Point& wp, Point& bp, Point& rp);

  // step 1
  void SortFoundPoints(vector<Point>& a, vector<Point>& b, vector<Point>& c, vector<int>& aa, vector<int>& ba, vector<int>& ca);

  Point MidPoint(Point a, Point b);

  Point confirmTarget(Point& currPoint);
  Point ContourCenter(vector <Point>& contour);
  Point ContourCenter(Rect contourBox);
  Point getRobot_XY();

  double calcDist(Point& a, Point& b);

  string intToString(int number);

  vector<Point> Perform_Contours(Mat& Objfound, Mat& Blobs, int minArea, Scalar boxColor, vector <int>& contArea);
};
 
#endif
