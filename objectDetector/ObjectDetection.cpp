#include "ObjectDetection.h"

using namespace cv;
using namespace std;

void ObjectDetection::setVariables(int addweight_whites_ , int addweight_blues_, int addweight_reds_,
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

Mat ObjectDetection::FindStuff(Mat& Input, bool showStitch){
  
  Point np  = Point(-1,-1);
  vector <int> wa, ba, ra;

  Robot_XY  = np;

  curr_wp   = np;
  curr_bp   = np;
  curr_rp   = np;

  currPoint = np;

  string whiteStr = "white";
  string blueStr  = "blue";
  string redStr   = "red";

  Mat Cache, getBlues, getReds, Objfound;

  Scalar getWhitesColor =  Scalar(255,255,255);
  Scalar getBluesColor  =  Scalar(255,100,0);
  Scalar getRedsColor   =  Scalar(0,100,255);

  Objfound = Input.clone();

  getBlues = Mat(Input.size(), Input.type());
  Perform_ImgAdjust(Input, getBlues, getReds);

  Cache = Perform_GetBlobWhite(Input, addweight_whites);
  Perform_ErodeDilate(Cache, whiteErodeArea, whiteMinArea, getWhitesColor);
  vector <Point> wpArray =  Perform_Contours(Objfound, Cache, whiteMinArea, getWhitesColor, wa);

  getBlues = Perform_GetBlobBlues(getBlues, addweight_blues);
  Perform_ErodeDilate(getBlues, blueErodeArea, blueMinArea, getBluesColor);
  vector <Point> bpArray = Perform_Contours(Objfound, getBlues, blueMinArea, getBluesColor, ba);

  getReds = Perform_GetBlobReds(getReds, addweight_reds);
  Perform_ErodeDilate(getReds, blueErodeArea, blueMinArea, getRedsColor);
  vector <Point> rpArray = Perform_Contours(Objfound, getReds, blueMinArea, getRedsColor, ra);

  SortFoundPoints(wpArray, bpArray, rpArray, wa, ba, ra);

  Point currPoint = CheckThreePoints(curr_wp, curr_bp, curr_rp);

  Robot_XY = CheckCurrPrev(currPoint, prevPoint, hitCount, hitCountlimit);

  ApplyCrosshair(currPoint, Objfound);
  
  Perform_UI(Objfound);

  if      (hitCount > hitCountlimit)  {hitCount = hitCountlimit;}
  else if (hitCount < 0)              {hitCount = 0;}

  if      (dc > cl) {dc = cl;}
  else if (dc <  0) {dc = 0;}

  if      (tc > cl) {tc = cl;}
  else if (tc <  0) {tc = 0;}
  
  if (Robot_XY.x <= 0 || Robot_XY.y <= 0){
    Robot_XY = np;
  }

  ApplyMarker(curr_wp, whiteStr, getWhitesColor, wa, Objfound);
  ApplyMarker(curr_bp, blueStr, getBluesColor, ba, Objfound);
  ApplyMarker(curr_rp, redStr, getRedsColor, ra, Objfound);
  
  ApplyString(Objfound, currPoint);

  cout << " hits " << hitCount << " | " << wp_count << " " << bp_count << " " << rp_count << endl;

  wa.clear();
  ba.clear();
  ra.clear();

  wpArray.clear();
  bpArray.clear();
  rpArray.clear();
  
  return Objfound;

}

// compare 2 points
Point ObjectDetection::CheckCurrPrev(Point& a, Point& b, int& count, int& countlimit){

  Point np = Point(-1,-1);

  if (b == np && a != np && count <=2 && a.y < nonsenseLine){
    count++;
    b = a;
    return np;
  }
  else if (a == np && b == np){
    count--;
    return np;
  }

  if (b != np && a != np && calcDist(a,b) < maxDist){
    b = a;
    count++;
  }
  else{
    b = a;
    count--;
    return np;
  }
  
  if (count >= 5){
    count++;
    return a;
  }
  
  return np;

}

// compare 6
Point ObjectDetection::CheckThreePoints(Point& wp, Point& bp, Point& rp){
  
  Point np = Point(-1,-1);

  wp = CheckOneOfThree(wp, prev_wp, wp_count, hitCountlimit);

  if      (wp_count >= hitCountlimit)  {wp_count = hitCountlimit;}
  else if (wp_count < 0)               {wp_count = 0;}

  bp = CheckOneOfThree(bp, prev_bp, bp_count, hitCountlimit);
  if      (bp_count >= hitCountlimit)  {bp_count = hitCountlimit;}
  else if (bp_count < 0)               {bp_count = 0;}

  rp = CheckOneOfThree(rp, prev_rp, rp_count, hitCountlimit);
  if      (rp_count >= hitCountlimit)  {rp_count = hitCountlimit;}
  else if (rp_count < 0)               {rp_count = 0;}

  if      (wp != np && bp == np && rp == np){
    return wp;
  }
  else if (wp == np && bp != np && rp == np){ 
    return bp;
  }
  else if (wp == np && bp == np && rp != np){
    return rp;
  }
  else if (wp != np && rp != np){
    if(calcDist(wp,rp) < maxDist){
      return MidPoint(wp,rp);
    }
    else{
      rp = np;
      return wp;
    }
  }
  else if (wp != np && bp != np){
    if (calcDist(wp,bp) < maxDist){
      return MidPoint(wp,bp);
    }
    else{
      bp = np;
      return wp;
    }
  }
  else if (rp != np && bp != np){
    if (calcDist(rp,bp) < maxDist){
      return MidPoint(rp,bp);
    }
    else{
      rp = np;
      return bp;
    }     
  }
  else if (wp != np && bp != np && rp != np){
    return np;
  }
  
  dc--;
  tc--;

  return np;
}

// compare 2
Point ObjectDetection::CheckOneOfThree(Point& a, Point& b, int& count, int& countlimit){
  
  Point np = Point(-1,-1);
  
  if (b == np && a != np && a.y < nonsenseLine && count == 0){
    count++;
    b = a;

    return a;
  }  
  else if (a != np && b != np){
    if (calcDist(a,b) < maxDist){
      count++;
      b = a;

      return a;
    }
    else{
      count--;
      a = np;
      b = np;

      return np;
    }
  }
  else if (a == np && b == np){
    count--;
    return np;
  }
  else{
    a = np;
    b = np;

    count--;

    return np;
  }
}

// compare up to 15

void ObjectDetection::SortFoundPoints(vector <Point>& a, vector <Point>& b, vector <Point>& c, vector <int>& aa, vector<int>& ba, vector<int>& ca){
  
  Point np = Point(-1,-1);

  if (a.empty()){
    curr_wp = np;
  }
  else if (a.size()==1){
    curr_wp = a[0];
  }
  else{
    if (!b.empty()){
      for (int i = 0; i < a.size(); i++){
	for (int k = 0; k < b.size(); k++){
	  if (calcDist(a[i], b[k]) < maxDist){
	    curr_wp = MidPoint(a[i], b[k]);
	    curr_bp = b[k];
	    dc++;

	    break;
	  }
	  else{
	    curr_wp = a[i];
	    curr_bp = b[k];
	  }
	}	
      }
    }
    else if (!c.empty()){
      for (int i = 0; i < a.size(); i++){
	for (int k = 0; k < c.size(); k++){
	  if (calcDist(a[i], c[k]) < maxDist){
	    curr_wp = MidPoint(a[i], c[k]);
	    curr_rp = c[k];
	    dc++;

	    break;
	  }
	  else{
	    curr_wp = a[i];
	    curr_rp = c[k];
	  }
	}
      }
    }
    else{
      for (int i = 0; i < a.size(); i++){
	if (calcDist(a[i], prev_wp) < maxDist){
	  curr_wp = a[i];
	}
      }
    }
  }

  if (b.empty()){
    curr_bp = np;
  }
  else if (b.size() == 1){
    curr_bp = b[0];
  }
  else{
    if (!b.empty()){
      for (int i = 0; i < b.size(); i++){
	for (int k = 0; k < c.size(); k++){

	  if (calcDist(b[i], c[k]) < maxDist){
	    curr_bp = MidPoint(b[i], c[k]);
	    curr_rp = c[k];

	    dc++;

	    break;
	  }
	  else{
	    curr_bp = b[i];
	    curr_rp = np;
	  }	        
	}
      }
    }
    else{
      for (int i = 0; i < b.size(); i++){
	if (calcDist(b[i], prev_bp) < maxDist){
	  curr_bp = b[i];
	}
      }
    }
  }

  if (c.empty()){
    curr_rp = np;
  }
  else if (c.size() == 1){
    curr_rp = c[0];
  }
  else{
    for (int i = 0; i < c.size(); i++){
      if (i == 0){
	curr_rp = c[i];
      }
      else if((i+1) < c.size() && ca[i] > ca[i+1]){
	curr_rp = c[i];
	ca[0] = ca[i];
      }
    }
  }
}

// need to write something sorting color points unto themselves

void ObjectDetection::Perform_ImgAdjust(Mat& getWhites, Mat& getBlues, Mat& getReds){

  vector<cv::gpu::GpuMat> channels;
  cv::gpu::GpuMat GPU_INPUT, GPU_getWhites, GPU_getBlues, GPU_getReds;

  GPU_INPUT.upload(getWhites);
  cv::gpu::GaussianBlur(GPU_INPUT, GPU_INPUT, Size(13,13), 1);
  cv::gpu::cvtColor(GPU_INPUT, GPU_getWhites, CV_BGR2Luv);

  split(GPU_getWhites, channels);

  GPU_getBlues  = channels[0].clone();
  GPU_getReds   = channels[1].clone();
  GPU_getWhites = channels[2].clone();
  
  cv::gpu::equalizeHist(GPU_getWhites, GPU_getWhites);

  Mat CPU_getWhites(GPU_getWhites);
  Mat CPU_getBlues(GPU_getBlues); 
  Mat CPU_getReds(GPU_getReds);
  
  getWhites = CPU_getWhites;
  getBlues  = CPU_getBlues;
  getReds   = CPU_getReds;

  channels.clear();

}

Mat ObjectDetection::Perform_GetBlobWhite(Mat& foo, int addweight){
  Mat bar = Scalar(255) - foo;
  threshold(bar, foo, addweight, 255, 0);
  return foo;
}

Mat ObjectDetection::Perform_GetBlobBlues(Mat& foo, int addweight){
  threshold(foo, foo, addweight, 255, 0);
  return foo;
}

Mat ObjectDetection::Perform_GetBlobReds(Mat& foo, int addweight){
  threshold(foo, foo, addweight, 255, 0);
  return foo;
}

void ObjectDetection::Perform_ErodeDilate(Mat& Blobs, int minErodeArea, int minColorArea, Scalar ObjColor){
  
  vector<vector< Point> > contours;

  double area;

  Mat temp_image;

  Mat erodeElement  = getStructuringElement(MORPH_RECT,Size(minErodeArea,minErodeArea));
  Mat dilateElement = getStructuringElement(MORPH_RECT,Size(20, 20));

  RemoveBySize(Blobs, minErodeArea);
  erode(Blobs, Blobs, erodeElement);

  dilate(Blobs, Blobs, dilateElement);
  Blobs.copyTo(temp_image);

  findContours(temp_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  if (!contours.empty()) {
    
    for (size_t i=0; i < contours.size(); ++i) {
      
      Rect tr1  =  boundingRect(Mat(contours[i]));
      Point foo = ContourCenter(tr1);
      area = contourArea(contours[i]);
     
      if (tr1.width > ShapeLimit || tr1.height > ShapeLimit){
	drawContours(Blobs, contours, i, Scalar(0), CV_FILLED, 8);
      }
      else if (tr1.width >= ShapeLimit && tr1.height >= ShapeLimit){
	drawContours(Blobs, contours, i, Scalar(0), CV_FILLED, 8);
      }
      else if (tr1.tl().x < 5 || tr1.tl().y < 5 || tr1.br().x + 5 >= Blobs.cols){
	drawContours(Blobs, contours, i, Scalar(0), CV_FILLED, 8);
      }
      else if (foo.y > 200 && area < minColorArea + 200){
	drawContours(Blobs, contours, i, Scalar(0), CV_FILLED, 8);
      }

    }
  }
  
  line(Blobs, Point(0,0), Point(Blobs.cols, 0), Scalar(0), 3, 4);
  line(Blobs, Point(0,0), Point(0, Blobs.rows), Scalar(0), 3, 4);
  line(Blobs, Point(0, Blobs.rows), Point(Blobs.cols, Blobs.rows), Scalar(0), 3, 4);
  line(Blobs, Point(Blobs.cols, 0), Point(Blobs.cols, Blobs.rows), Scalar(0), 3, 4);

  contours.clear();
}

vector<Point> ObjectDetection::Perform_Contours(Mat& Objfound, Mat& Blobs, int minArea, Scalar boxColor, vector <int>& contArea){

  vector<Point> foundPoints;
  vector< vector<Point> > contours;

  Rect tr1, tr2;
  int area;

  findContours(Blobs, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  if (contours.size() > 0 && contours.size() < 7){

    for(int i = 0; i < contours.size(); i++){

      int nxWidth  = 100;
      int nyHeight = 100;

      approxPolyDP(Mat(contours[i]), contours[i], 0, true);
      area = contourArea(contours[i], false);
      tr1 =  boundingRect(Mat(contours[i]));

      if (tr1.width >= ShapeLimit && tr1.height >= ShapeLimit){
	area = 0;
      }
      else if (tr1.tl().x < 5 || tr1.tl().y < 5 || tr1.br().x + 5 >= Blobs.cols){
	area = 0;
      }      

      if (area >= minArea){

	Point oc = ContourCenter(tr1);
	
	drawContours(Objfound, contours, i, colorBlack,   2, 8);
	drawContours(Objfound, contours, i, boxColor, 1.5, 8);

	contArea.push_back(area);
	foundPoints.push_back(oc);
	
      }
    } 
  }

  return foundPoints;

}

void ObjectDetection::Perform_UI(Mat& Objfound){

  line(Objfound, Point(0, nonsenseLine), Point(Objfound.cols, nonsenseLine), Scalar(colorGreen), 1, 4);
  rectangle (Objfound, Point(0, 0), Point(119, 22), Scalar(0), -1, 8, 0);

  putText(Objfound, "Team MAXed-Out", Point(1,10), 1, 0.8,colorGreen, 1, 8);
  putText(Objfound, "NASA SRR 2016", Point(1,21), 1, 0.8,colorGreen, 1, 8);

  niceText(Objfound, "Jon Lee", Point(585,479), colorGreen);

  rectangle(Objfound, Point(0, Objfound.rows), Point(198, Objfound.rows - 45),Scalar(0), -1, 8, 0);
  
  putText(Objfound, "Whites :" ,Point(1,445), 1, 0.8, Scalar(255,255,255), 1, 8);
  putText(Objfound, "Blues  :" ,Point(1,460), 1, 0.8, Scalar(255,255,255), 1, 8);
  putText(Objfound, "Reds",Point(1,475), 1, 0.8, Scalar(255,255,255), 1, 8);
  putText(Objfound, ":",Point(48,475), 1, 0.9, Scalar(255,255,255), 1, 8);
 
  putText(Objfound, "area : ", Point(118,445), 1, 0.8, Scalar(255,255,255), 1, 8);
  putText(Objfound, "area : ", Point(118,460), 1, 0.8, Scalar(255,255,255), 1, 8);
  putText(Objfound, "area : ", Point(118,475), 1, 0.8, Scalar(255,255,255), 1, 8);

}

void ObjectDetection::ApplyString(Mat& Objfound, Point& currPoint){

  if (hitCount < hitCountlimit){
    string progremaxDist;

    for (int i = 0; i < hitCount; i++){
      progremaxDist.append(".");
    }

    putText(Objfound, progremaxDist, Point (currPoint.x-50, currPoint.y-54), 1, 2, Scalar(0), 3, 8);
    putText(Objfound, progremaxDist, Point (currPoint.x-50, currPoint.y-54), 1, 2, Scalar(0,255,0), 2, 8);
    HAILHYDRA = false;
  }
  else if (Robot_XY != Point(-1,-1) && hitCount+3 >= hitCountlimit){
    if (HAILHYDRA == false && currStr == false){
      std::mt19937 rng;
      rng.seed(std::random_device()());
      std::uniform_int_distribution<std::mt19937::result_type> dist6(1,2);
      int foo = dist6(rng);
    
      if (foo == 1) {HAILHYDRA = true;}
      else          {HAILHYDRA = false;}
      currStr = true;
    }

    if(HAILHYDRA == true){
      niceText(Objfound, "    HAIL HYDRA", Point(currPoint.x-70, currPoint.y-104), Scalar(0,0,255));
      niceText(Objfound, " \\0/ \\0/  \\0/ \\0/", Point(currPoint.x-70, currPoint.y-90), Scalar(0,0,255));
      niceText(Objfound, " \\0/ \\0/  \\0/ \\0/", Point(currPoint.x-70, currPoint.y-78), Scalar(0,0,255));
      niceText(Objfound, " \\0/ \\0/  \\0/ \\0/", Point(currPoint.x-70, currPoint.y-66), Scalar(0,0,255));
      niceText(Objfound, " \\0/ \\0/  \\0/ \\0/", Point(currPoint.x-70, currPoint.y-54), Scalar(0,0,255));
    }
    else{
      niceText(Objfound, "TARGET LOCKED", Point(currPoint.x-52, currPoint.y-54), Scalar(255,200,0));
    }
  }
  else{
    HAILHYDRA = false;
    currStr = false;
  }
}

void ObjectDetection::ApplyMarker(Point& drawPoint, string& ObjName, Scalar color, vector <int>& FinalArea, Mat& Objfound){

  if(drawPoint != Point(-1,-1)){

    int x = drawPoint.x;
    int y = drawPoint.y;

    if (color == Scalar(255,255,255) && !FinalArea.empty()){
      putText(Objfound, " " + intToString(x)+","+intToString(y) + " ",Point(53,445), 1, 0.8, colorGreen, 1, 8);
      putText(Objfound, intToString(FinalArea[0]), Point(163,445), 1, 0.8, colorGreen, 1, 8);
    }
    else if (color == Scalar(255,100,0) && !FinalArea.empty()){
      putText(Objfound, " " + intToString(x)+","+intToString(y), Point(53,460), 1, 0.8, colorGreen, 1, 8);
      putText(Objfound, intToString(FinalArea[0]), Point(163,460), 1, 0.8, colorGreen, 1, 8);
    }
    else if (color == Scalar(0,100,255) && !FinalArea.empty()){
      putText(Objfound, " " + intToString(x)+","+intToString(y),Point(53,475), 1, 0.8, colorGreen, 1, 8);
      putText(Objfound, intToString(FinalArea[0]), Point(163,475), 1, 0.8, colorGreen, 1, 8);
    }
  }
}

void ObjectDetection::ApplyCrosshair(Point& drawPoint, Mat& Objfound){
  if (drawPoint != Point(-1,-1)){

    int xSpacing = 7;
    int ySpacing = 7;
    int xLength = 20;
    int yLength = 20;

    int x = drawPoint.x;
    int y = drawPoint.y;

    rectangle(Objfound,Point(x,y-ySpacing),Point(x, y-yLength), colorBlack,3, CV_AA, 0);
    rectangle(Objfound,Point(x,y+ySpacing),Point(x, y+yLength), colorBlack,3, CV_AA, 0);
    rectangle(Objfound,Point(x-xSpacing, y),Point(x-xLength, y), colorBlack,3, CV_AA, 0);
    rectangle(Objfound,Point(x+xSpacing, y),Point(x+xLength, y), colorBlack,3, CV_AA, 0);

    rectangle(Objfound,Point(x,y-ySpacing),Point(x, y-yLength), colorGreen,2, CV_AA, 0);
    rectangle(Objfound,Point(x,y+ySpacing),Point(x, y+yLength), colorGreen,2, CV_AA, 0);
    rectangle(Objfound,Point(x-xSpacing, y),Point(x-xLength, y), colorGreen,2, CV_AA, 0);
    rectangle(Objfound,Point(x+xSpacing, y),Point(x+xLength, y), colorGreen,2, CV_AA, 0);

    rectangle(Objfound, Point(x-50, y-50), Point(x+50, y+50), Scalar(0), 3, CV_AA, 0);
    rectangle(Objfound, Point(x-50, y-50), Point(x+50, y+50), colorGreen, 2, CV_AA, 0);

  }
}

void ObjectDetection::RemoveBySize(Mat& a, int minArea){
  Mat temp = a.clone();
  vector<vector <Point> > contours;
 
  findContours(temp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  for (int i = 0; i < contours.size(); i++){
    double area = contourArea(contours[i]);
    if (area <= minArea){
      drawContours(a, contours, i, Scalar(0), CV_FILLED, 8);
    }
  }
}

double ObjectDetection::calcDist(Point& a, Point& b){
  
  int x1 = a.x; int y1 = a.y;
  int x2 = b.x; int y2 = b.y;

  double distance = norm(a - b);
    
  return distance;
}

string ObjectDetection::intToString(int number){
  std::stringstream maxDist;
  maxDist << number;

  return maxDist.str();
}

Point ObjectDetection::getRobot_XY(){
  return Robot_XY;
}

Point ObjectDetection::MidPoint(Point a, Point b){  
  return Point((a.x+b.x)/2, (a.y+b.y)/2);
}

void ObjectDetection::niceText(Mat& Objfound, string str, Point textStart, Scalar color){
  putText(Objfound, str, textStart, 1, 0.8, Scalar(0), 2, 8);
  putText(Objfound, str, textStart, 1, 0.8, color, 1, 8);
}

Point ObjectDetection::ContourCenter(vector <Point>& contour){

  Rect a = boundingRect(Mat(contour));
  Point objCenter = a.br() + a.tl();

  objCenter.x /= 2;
  objCenter.y /= 2;

  return objCenter;
}

Point ObjectDetection::ContourCenter(Rect a){

  Point ap = Point(a.br().x - a.width/2, a.br().y - a.height/2);

  return ap;
}
										
