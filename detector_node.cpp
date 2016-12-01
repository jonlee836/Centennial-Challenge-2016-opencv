#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <srrc_detection/DetectorResult.h>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>
#include <chrono>

#include "jon/ObjectDetection.h"

// Unless you are using ROS just use the main.cpp file in the opencv folder
// http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages

ros::Publisher res_pub;
image_transport::Publisher img_pub;

ObjectDetection od;

image_geometry::PinholeCameraModel cam_model_;
tf::TransformListener *tf_listener_;
tf::StampedTransform tCamToBase;

/**
 * O - base_footprint, on the ground
 * C - camera
 * R - point somewhere on the ray corresponding to the pixel (x,y)
 * P - point on the ground on the ray corresponding to the pixel (x,y)
 */
tf::Vector3 projectPixelOnGround(int x,int y) {
	//ROS_INFO("x %d y %d", x, y);

	tf::Vector3 OC = tCamToBase.getOrigin();

	//ROS_INFO("OC %.3f %.3f %.3f", OC.getX(), OC.getY(), OC.getZ());

	Point3d CR_cam = cam_model_.projectPixelTo3dRay(Point2i(x, y));          // R in camera frame
	tf::Vector3 OR = tCamToBase(tf::Vector3(CR_cam.x, CR_cam.y, CR_cam.z));  // R in base frame

	//ROS_INFO("OR %.3f %.3f %.3f", OR.getX(), OR.getY(), OR.getZ());

	tf::Vector3 CR = OR - OC;

	//ROS_INFO("CR %.3f %.3f %.3f", CR.getX(), CR.getY(), CR.getZ());

	tf::Vector3 CP = CR / fabs(CR.getZ()) * fabs(OC.getZ()); // extend CR to the ground

	//ROS_INFO("CP %.3f %.3f %.3f", CP.getX(), CP.getY(), CP.getZ());
	//ROS_INFO("OP %.3f %.3f %.3f", (OC + CP).getX(), (OC + CP).getY(), (OC + CP).getZ());
	return OC + CP;
}

double distanceOnGround(int x1,int y1,int x2,int y2) {
	return (projectPixelOnGround(x1,y1) - projectPixelOnGround(x2,y2)).length();
}

//void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//  try {
//    tf_listener_->lookupTransform("base_footprint", "right_camera_optical_frame", ros::Time(0), tCamToBase);
//  } catch(tf::TransformException& ex) {
//    ROS_ERROR("Transform error in callback: %s", ex.what());
//  }
//  cam_model_.fromCameraInfo(info_msg);

  // Test for the distance function
  //ROS_INFO("distance %.3f", distanceOnGround(177,255,234,254));

  try
  {
    auto t0 = std::chrono::high_resolution_clock::now();
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    
    // ===== do some detection =========
    cv::Mat c = img.clone();
    auto t1 = std::chrono::high_resolution_clock::now();
    cv::Mat outImg = od.FindStuff(c, false); // don't show stitch
    auto t2 = std::chrono::high_resolution_clock::now();
 	Point found = od.getRobot_XY();

    // publish result
    srrc_detection::DetectorResult result;
    result.x = found.x;
    result.y = found.y;
    result.hits = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
    res_pub.publish(result);
    
    // optional - debug image
    if (result.x >= 0) {
    	cv::circle(outImg, cv::Point(result.x,result.y), 20, Scalar(0,0,255), 4);
    }
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outImg).toImageMsg();
    img_pub.publish(img_msg);
    auto t3 = std::chrono::high_resolution_clock::now();
    int total_time = std::chrono::duration_cast<std::chrono::milliseconds>(t3-t0).count();
    if (total_time >= 100) {
        ROS_ERROR("publish result %d,%d hits %d, total time %d ms", result.x, result.y, result.hits, total_time);
    } else {
        if (total_time >= 90) {
            ROS_WARN("publish result %d,%d hits %d, total time %d ms", result.x, result.y, result.hits, total_time);
        } else {
            ROS_INFO("publish result %d,%d hits %d, total time %d ms", result.x, result.y, result.hits, total_time);
        }
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detector_node");
  ros::NodeHandle nh;
  
  ROS_INFO("OpenCV version from headers: " CV_VERSION);

  res_pub = nh.advertise<srrc_detection::DetectorResult>("detector_result", 1);
  
//  tf_listener_ = new tf::TransformListener(nh, ros::Duration(3));

  image_transport::ImageTransport it(nh);

  img_pub = it.advertise("debug_image", 1);
  
  //image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback, image_transport::TransportHints("compressed"));
  //image_transport::Subscriber sub = it.subscribe("/right_cam/camera/image", 1, imageCallback, image_transport::TransportHints("compressed"));
  //image_transport::CameraSubscriber sub = it.subscribeCamera("image", 1, imageCallback, image_transport::TransportHints("raw"));
  image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);

  ros::spin();
}

