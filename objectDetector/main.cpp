#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/gpu/gpu.hpp"

#include <time.h> 
#include <cmath>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <stdlib.h>

#include "ObjectDetection.h"

using namespace cv;
using namespace std;
using namespace std::chrono;

int FPS = 30;

int whiteMinArea = WHITE_MIN_AREA;
int whiteErodeArea = WHITE_ERODE_AREA;

int blueMinArea = BLUE_MIN_AREA;
int blueErodeArea = BLUE_ERODE_AREA;

int addweight = ADDWEIGHT;

const string trackbar1 = "GUI TRACKBAR";

void on_trackbar(int, void*){}
void gui(){


  createTrackbar("FPS "                       , trackbar1, &FPS,          30, on_trackbar);

  createTrackbar("min white obj area "        , trackbar1, &whiteMinArea,   2000, on_trackbar);
  createTrackbar("min white obj erode "  , trackbar1, &whiteErodeArea, 2000, on_trackbar);

  createTrackbar("min blue obj area "         , trackbar1, &blueMinArea,   2000, on_trackbar);
  createTrackbar("min blue obj erode "   , trackbar1, &blueErodeArea, 2000, on_trackbar);
 
  createTrackbar("add float "                 , trackbar1, &addweight,    1000, on_trackbar);
  
}

void startUp(){
  
  cout << endl;
  cout << "NOTE : The fps trackbar goes up to 30, but it's really 13-15 on the jetson tk1." << endl;
  cout << "Set the fps below 15 to begin lowering the fps."<<endl;
  cout << "Currently learning the cuda/gpu bindings within linux4tegra to increase the time to process each frame." << endl << endl;  

  cout << "press 1 to process the current frame or to pause the stream" << endl;
  cout << "press 2 to process all given frames 1 at a time " << endl;
  cout << "press n to go forward 1 frame" << endl;
  cout << "press b to go back 1 frame" << endl << endl;;

  cout << "A point of -1,-1 means nothing found " << endl;
  cout << "RPI cam settings"<< endl;
  cout << "awb = auto, exposure = auto, resolution 640x480" << endl;
  cout << "image effect saturation seemed to have good results as well" << endl;
  
}

void runObjDetection(Mat& Input, string name, ObjectDetection& go){
  Input = imread(name);	       
  go.setVariables(addweight, whiteMinArea, whiteErodeArea, blueMinArea, blueErodeArea);
  // go.FindStuff(Input);
  imshow("objfound", go.FindStuff(Input));
}


int main(int argc, char** argv)
{

  namedWindow(trackbar1, 0);
  gui();
  startUp();

  Mat Input;

  ObjectDetection go;

  bool Allimg = false;
  int start_s = clock();
  
  double totaltime;
  int ImgNumb;

  for(int i = 1; i < argc;){
    
    string name = argv[i];
    // runObjDetection(Input, name, go);
   
    while (true){

      int k = waitKey(1);
      
      if (char(k) == '1'){ // re-process current frame
	Allimg = false;       
	string name = argv[i];
	int start_s = clock();

	runObjDetection(Input, name, go);

	int stop_s = clock();
	double execTime = (stop_s-start_s)/double(CLOCKS_PER_SEC);
	Point test = go.getRobot_XY();

	totaltime = totaltime + execTime;

	cout << execTime << " return " << test;

	// i++;

	// ImgNumb = i;
      }
      else if (char(k) == '2')             {Allimg = true;} 
      else if (char(k) == 'b' && i > 1)    {i--; break;}
      else if (char(k) == 'n' && i < argc) {i++; break;}      
      else if (char(k) == 'q')             {i = argc; break;}
      else if (Allimg == true){
	string name = argv[i];
	int start_s = clock();

	runObjDetection(Input, name, go);

	int stop_s = clock();
	double execTime = (stop_s-start_s)/double(CLOCKS_PER_SEC);
	Point test = go.getRobot_XY();

	totaltime = totaltime + execTime;

	cout << execTime << " return " << test;

	i++;

	ImgNumb = i;

	break;
      }
    }
  }

  cout << "total images : " <<  ImgNumb << " time " << totaltime << " seconds " << " fps " << ImgNumb/totaltime << endl;
    
  return 0;
}
