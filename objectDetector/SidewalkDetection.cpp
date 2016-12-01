#include "SidewalkDetection.h"

using namespace cv;
using namespace std;

void SidewalkDetection::setVariables(int addweight_whites_ , int addweight_blues_, int addweight_reds_,
				   int whiteMinArea_, int whiteErodeArea_,
				   int blueMinArea_, int blueErodeArea_)
{ 
  addweight_whites = addweight_whites_;
  addweight_blues  = addweight_blues_;
  addweight_reds   = addweight_reds_;

  whiteMinArea = whiteMinArea_;
  whiteErodeArea = whiteErodeArea_;

  blueMinArea = blueMinArea_;
  blueErodeArea = blueErodeArea_;
}

Mat SidewalkDetection::FindStuff(Mat& Input, bool showStitch){
  
  Robot_Cpoint = Point(-1, -1);
  Robot_Lpoint = Point(-1, -1);
  Robot_Rpoint = Point(-1, -1);

  string whiteStr = "white";
  string blueStr  = "blue";
  string redStr   = "red";

  Mat Cache, getBlues, getReds, Objfound;

  Scalar getWhiteColor = Scalar(255,255,255); // white box color
  Scalar getBluesColor = Scalar(255,100,0);   // green box color
  Scalar getRedsColor  = Scalar(0,100,255);

  Objfound = Input.clone();

  getBlues = Mat(Input.size(), Input.type());
  Perform_ImgAdjust(Input, getBlues, getReds);

  Cache = Perform_GetBlobWhite(Input, addweight_whites);
  Perform_ErodeDilate(Cache, whiteErodeArea, whiteMinArea, getWhiteColor);

  imshow("after erode dilate", Cache);
  Perform_UI(Objfound);
  Perform_Contours(Objfound, Cache, whiteMinArea, getWhiteColor);
  
  return Objfound;
}

void SidewalkDetection::Perform_ImgAdjust(Mat& getWhites, Mat& getBlues, Mat& getReds){

  vector<cv::gpu::GpuMat> channels, channels2;
  cv::gpu::GpuMat GPU_INPUT, GPU_getWhites, GPU_getBlues, GPU_getReds;

  GPU_INPUT.upload(getWhites);

  cv::gpu::cvtColor(GPU_INPUT, GPU_getWhites, CV_BGR2HSV);

  split(GPU_getWhites, channels);
  GPU_getWhites = channels[1].clone();

  // cv::gpu::equalizeHist(GPU_getWhites, GPU_getWhites);

  Mat CPU_getWhites(GPU_getWhites);

  getWhites = CPU_getWhites.clone();
 
  channels.clear();
  channels2.clear();

}

Mat SidewalkDetection::Perform_GetBlobWhite(Mat& foo, int addweight){

  cv::gpu::GpuMat GPU_foo;
  GPU_foo.upload(foo);
  cv::gpu::GaussianBlur(GPU_foo, GPU_foo, Size(15,15), 0);
  Mat bar(GPU_foo);
  // Mat bar = foo.clone();
  // medianBlur(bar, foo, 7);

  bar = Scalar(255) - foo;
  threshold(bar, foo, addweight, 255, 0);
  imshow("v", foo);
  return foo;
}

void SidewalkDetection::Perform_ErodeDilate(Mat& Blobs, int minErodeArea, int minColorArea, Scalar ObjColor){
  
  vector<vector< Point> > contours_poly;
  vector<int> small_blobs;

  double contour_area;

  Mat temp_image;

  Mat erodeElement = getStructuringElement(MORPH_ELLIPSE,Size(5,5));
  Mat dilateElement = getStructuringElement(MORPH_ELLIPSE,Size(5,5));

  erode(Blobs, Blobs, erodeElement);
  Blobs.copyTo(temp_image);

  findContours(temp_image, contours_poly, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  if (!contours_poly.empty()) {
    
    for (size_t i=0; i < contours_poly.size(); ++i) {
      
      Rect tempR =  boundingRect(Mat(contours_poly[i]));
      Point foo = ContourCenter(tempR);
      contour_area = contourArea(contours_poly[i]);
     
      if (contour_area < 3000 && foo.y > 150){	
	small_blobs.push_back(i);
      }
      if (tempR.width < 90 || tempR.height < 90){
	small_blobs.push_back(i);
      }      
      if (contour_area <= minErodeArea){
	small_blobs.push_back(i);
      }
    }
  }

  for (size_t i = 0; i < small_blobs.size(); ++i) {
    drawContours(Blobs, contours_poly, small_blobs[i], Scalar(0), CV_FILLED, 8);
  }

  // this gets rid of the annoying outlying blobs at the edge of an image frame and 1-3 in area.
  // Scalar(0) is black

  line(Blobs, Point(0,0), Point(Blobs.cols, 0), Scalar(0), 3, 4);
  line(Blobs, Point(0,0), Point(0, Blobs.rows), Scalar(0), 3, 4);
  line(Blobs, Point(0, Blobs.rows), Point(Blobs.cols, Blobs.rows), Scalar(0), 3, 4);
  line(Blobs, Point(Blobs.cols, 0), Point(Blobs.cols, Blobs.rows), Scalar(0), 3, 4);

  dilate(Blobs, Blobs, dilateElement);

  contours_poly.clear();
  small_blobs.clear();

  // make all blobs bigger regardless of size. this is to counter the erode from before
}

void SidewalkDetection::Perform_Contours(Mat& Objfound, Mat& Blobs, int minArea, Scalar bigBoxColor){

  // sort contours and determine if a shape is an object to pick up

  Point result = Point(-1,-1);
  int width = Blobs.cols; int height = Blobs.rows;
  Scalar color = bigBoxColor;
  Scalar bigbox = color;

  vector< vector<Point> > shapes;
  vector< vector<Point> > contours;

  vector <Point> Sidewalk, Sidewalk1, Sidewalk2;

  findContours(Blobs, shapes, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  if (shapes.size() > 0){
    contours.resize(shapes.size());
    for (int i = 0; i < shapes.size(); i++){
      approxPolyDP(shapes[i], contours[i], 3, true);
    }

    SortContoursBy_Size(contours);

    Rect tempR = boundingRect(Mat(contours[0]));

    int change;
    Point corner, left, right;

    Sidewalk = contours[0];
    Sidewalk1 = contours[0];
    Sidewalk2 = contours[0];

    SortContourBy_Y(Sidewalk);
    Robot_Cpoint = Sidewalk[0];

    SortContourBy_X_right(Sidewalk1);
    change = Sidewalk1[0].x;
    for (int i = 0; i < Sidewalk1.size(); i++){
      if (Sidewalk1[i].x < change){
	Robot_Rpoint = Sidewalk1[i];
	break;
      }
    }

    SortContourBy_X_left(Sidewalk2);
    change = Sidewalk2[0].x;
    for (int i = 0; i < Sidewalk2.size(); i++){
      if (Sidewalk2[i].x > change){
	Robot_Lpoint = Sidewalk2[i];
	break;
      }
    }
   
    drawContours(Objfound, contours, 0, Scalar(0,255,0), 1.5, 8);
    
    // rectangle(Objfound, tempR.tl(), tempR.br(), Scalar(255,150,0), 2, 8, 0);

    circle(Objfound, Robot_Cpoint, 4, Scalar(0,0,255), -1, 8, 0);
    circle(Objfound, Robot_Rpoint, 4, Scalar(255,0,0), -1, 8, 0);
    circle(Objfound, Robot_Lpoint, 4, Scalar(0,255,0), -1, 8, 0);

    putText(Objfound, " " +intToString(Robot_Lpoint.x)+","+intToString(Robot_Cpoint.y), Point(90,445), 1, 0.8, colorGreen, 1, 8);
    putText(Objfound, " " +intToString(Robot_Rpoint.x)+","+intToString(Robot_Cpoint.y), Point(90,460), 1, 0.8, colorGreen, 1, 8);
    putText(Objfound, " " +intToString(Robot_Cpoint.x)+","+intToString(Robot_Cpoint.y), Point(90,475), 1, 0.8, colorGreen, 1, 8);
    
  }

  cout << Sidewalk.size() << " Corner " << Robot_Cpoint << " Left " << Robot_Lpoint << " Right " << Robot_Rpoint << endl;
}

void SidewalkDetection::SortContoursBy_Size(vector <vector <Point> >& contours){
 
  double biggest;

  while(true){

    int sortCount = 0;

    for (int i = 0; i < contours.size()-1; i++){ // sort by size largest to smallest

      vector<Point> contourHold;
      double area1 = contourArea(contours[i], false);   
      double area2 = contourArea(contours[i+1], false);
      
      if (area1 < area2){
	biggest = area2;
	contourHold = contours[i];
	contours[i] = contours[i+1];
	contours[i+1] = contourHold;
	sortCount++;
      }
    }
    if (sortCount == 0){
      break;
    }
  }

}

void SidewalkDetection::SortContourBy_Y(vector <Point>& contour){
  
  struct myclass{
    bool operator() (cv::Point pt1, cv::Point pt2) {return (pt1.y > pt2.y);}    
  } mySort;

  std::sort(contour.begin(), contour.end(), mySort);
}

void SidewalkDetection::SortContourBy_X_left(vector <Point>& contour){
  
  struct myclass{
    bool operator() (cv::Point pt1, cv::Point pt2) {return (pt1.x < pt2.x);}    
  } mySort;

  std::sort(contour.begin(), contour.end(), mySort);
}

void SidewalkDetection::SortContourBy_X_right(vector <Point>& contour){
  
  struct myclass{
    bool operator() (cv::Point pt1, cv::Point pt2) {return (pt1.x > pt2.x);}    
  } mySort;

  std::sort(contour.begin(), contour.end(), mySort);
}

double SidewalkDetection::calcDist(Point& prevPoint, Point& currPoint){
  
  int x1 = prevPoint.x; int y1 = prevPoint.y;
  int x2 = currPoint.x; int y2 = currPoint.y;

  double distance = norm(prevPoint - currPoint);
    
  return distance;
}

string SidewalkDetection::intToString(int number){
  std::stringstream ss;
  ss << number;

  return ss.str();
}

Point SidewalkDetection::ContourCenter(vector <Point>& contour){
  Rect a = boundingRect(Mat(contour));
  Point objCenter = a.br() + a.tl();

  objCenter.x /= 2;
  objCenter.y /= 2;

  return objCenter;
}

Point SidewalkDetection::ContourCenter(Rect contourBox){
  Point objCenter = contourBox.br() + contourBox.tl();
  objCenter.x /= 2;
  objCenter.y /= 2;

  return objCenter;
}

void SidewalkDetection::Perform_UI(Mat& Objfound){

  rectangle (Objfound, Point(0, 0), Point(118, 22), Scalar(0), -1, 8, 0);

  putText(Objfound, "Team MAXed-Out", Point(1,10), 1, 0.8,colorGreen, 1, 8);
  putText(Objfound, "NASA SRR 2016", Point(1,21), 1, 0.8,colorGreen, 1, 8);

  niceText(Objfound, "Jon Lee", Point(585,479), colorGreen);

  rectangle(Objfound, Point(0, Objfound.rows), Point(150, Objfound.rows - 45), colorBlack, -1, 8, 0);
  
  putText(Objfound, "Left Point    " ,Point(1,445), 1, 0.8, colorWhite, 1, 8);
  putText(Objfound, "Right Point   "   ,Point(1,460), 1, 0.8, colorWhite, 1, 8);
  putText(Objfound, "Corner Point  "  ,Point(1,475), 1, 0.8, colorWhite, 1, 8);
}

void SidewalkDetection::niceText(Mat& Objfound, string str, Point textStart, Scalar color){
  putText(Objfound, str, textStart, 1, 0.8, Scalar(0), 2, 8);
  putText(Objfound, str, textStart, 1, 0.8, color, 1, 8);
}

Point SidewalkDetection::getRobot_Rpoint(){
  return Robot_Rpoint;
}

Point SidewalkDetection::getRobot_Lpoint(){
  return Robot_Lpoint;
}

Point SidewalkDetection::getRobot_Mpoint(){
  return Robot_Cpoint;
}
