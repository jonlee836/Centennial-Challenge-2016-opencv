#ifndef SIDEWALKDETECTION_H
#define SIDEWALKDETECTION_H

#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cmath>
#include <algorithm>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>

using namespace cv;
using namespace std;

const int ADDWEIGHT_WHITES = 70;
const int ADDWEIGHT_BLUES = 250;
const int ADDWEIGHT_REDS  = 252;

const int WHITE_MIN_AREA = 30;
const int WHITE_ERODE_AREA = 50;
const int BLUE_MIN_AREA = 20;
const int BLUE_ERODE_AREA = 8;

class SidewalkDetection
{
  
 private :
  
  int B_adjust, G_adjust, R_adjust, minArea, whiteMinArea, whiteErodeArea;
  int addweight_whites, addweight_blues, addweight_reds;

  int blueMinArea, blueErodeArea;
  
  int wa = 0;
  int ba = 0;
  int ra = 0;

  int hitCount        = 0;
  int hitCountlimit   = 9;
  int hitCountDist    = 15;

  int stillCount      = 0;
  int stillDistLimit  = 10;
  int stillCountLimit = 15;

  int addweight, whiteArea, colorArea;
  
  Point Robot_Lpoint = Point(-1,-1);
  Point Robot_Cpoint = Point(-1,-1);
  Point Robot_Rpoint = Point(-1,-1);

  Point prev_wp      = Point(-1,-1);
  Point prev_bp      = Point(-1,-1);
  Point prev_rp      = Point(-1,-1);
  
  Point prevPoint    = Point(-1,-1);
  Point currPoint    = Point(-1,-1);
  Point disTolerance = Point(-1,-1);

  Mat Objfound;

  bool HAILHYDRA = false;
  bool currStr = false;

  Scalar colorBlack = Scalar(0);
  Scalar colorWhite = Scalar(255,255,255);
  Scalar colorGreen = Scalar(0,255,0);

 public :

  SidewalkDetection() {
	
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

  void Perform_UI(Mat& Objfound);
  void Perform_ErodeDilate(Mat& Blobs, int minErodeArea, int minColorArea, Scalar ObjColor);
  void ApplyMarker(Point& drawPoint, string& ObjName, Scalar color, int& FinalArea, Mat& Objfound);
  void ApplyCrosshair(Point& drawPoint, Mat& Objfound);
  void adjustImg(Mat& Input);
  void ApplyString(Mat& Objfound, Point& currPoint);
  void niceText(Mat& Objfound, string str, Point textStart, Scalar color);
  void PointCheck(Point a, Point b);
  void Perform_Contours(Mat& Objfound, Mat& Blobs, int minArea, Scalar bigBoxColor);

  Point PointToPass(Point& wp, Point& bp, Point& rp);
  Point confirmTarget(Point& currPoint);
  Point ContourCenter(vector <Point>& contour);
  Point ContourCenter(Rect contourBox);

  Point FindCorner_Point(vector <Point>& sidewalk);

  Point getRobot_Lpoint();
  Point getRobot_Mpoint();
  Point getRobot_Rpoint();

  void SortContourBy_Y(vector <Point>& contour);
  void SortContourBy_X_left(vector <Point>& contour);
  void SortContourBy_X_right(vector <Point>& contour);

  void SortContoursBy_Size(vector <vector<Point> >& contours);

  double calcDist(Point& prevPoint, Point& currPoint);

  string intToString(int number);

};
 
#endif
