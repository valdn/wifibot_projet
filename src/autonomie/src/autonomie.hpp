#ifndef AUTONOMIE_HPP
#define AUTONOMIE_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

typedef struct{
  float lx = 0;
  float az = 0;
  ros::Time temps;
} vel_t;

static const std::string OPENCV_WINDOW = "Image window";
const double pi = 3.14159265358979323846;

class Autonomie
{
  public:
  ros::NodeHandle nh_;
  /*nav_msgs::Odometry odom;
  ros::Subscriber odom_sub_ ;*/
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher cmd_vel_pub;
  int current = -1;
  float AngularSpeed, LinearSpeed;
  long unsigned int compt=0;
  std::vector<vel_t> tabvel;
  cv_bridge::CvImagePtr cv_ptr;
  geometry_msgs::Twist msg_cmd_vel;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f> > corners;
  cv::Ptr<cv::aruco::DetectorParameters> parameters;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Ptr<cv::aruco::DetectorParameters> params;
  std::string filename;
  bool readOk;
  cv::Mat cameraMatrix, distCoeffs;
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::Vec3d rvec;
  cv::Vec3d tvec;
  bool redoStart=false;
  ros::Time doTime,redoTime, temps;



  Autonomie();
  ~Autonomie();
  bool readCameraParameters(std::string filename, cv::Mat &camMatrix, cv::Mat &distCoeffs);
  void Calibrate_Camera();
  void DessinAxis();
  void Get_ArUcO_Position();
  void Course_Recording();
  void Course_Redo();
  void Course_Run();
  void Update_GUI();
  void FollowAuto();
  void affichertab();
  void ResetSpeed();
  void Move(float linear, float angular);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
};

#endif