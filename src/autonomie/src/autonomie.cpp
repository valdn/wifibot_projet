#include "autonomie.hpp"

Autonomie::Autonomie():it_(nh_)
{
  // Subscrive to input video feed and publish output video feed
  image_sub_ = it_.subscribe("usb_cam/image_raw", 1,&Autonomie::imageCb, this);
  /*odom_sub_ = nh_.subscribe("odom", 1,&ImageConverter::odomCb, this);*/

  // Advertise velocity to Wifibot low-level program
  cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  Calibrate_Camera();

  cv::namedWindow(OPENCV_WINDOW);
}

Autonomie::~Autonomie()
{
  cv::destroyWindow(OPENCV_WINDOW);
}

/* void odomCb(const nav_msgs::Odometry& _odom){
    odom=_odom;
  }*/

void Autonomie::imageCb(const sensor_msgs::ImageConstPtr& msg){
  try
  {  
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    FollowAuto();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    throw;
  }
}


bool Autonomie::readCameraParameters(std::string filename, cv::Mat &camMatrix, cv::Mat &distCoeffs) {
  cv::FileStorage fs;
  fs.open(filename, cv::FileStorage::READ);
  if(!fs.isOpened()){
    return false;
  }
  fs["camera_matrix"] >> camMatrix;
  fs["distortion_coefficients"] >> distCoeffs;
  return true;
}

// Calibrate camera with premade file.yaml
void Autonomie::Calibrate_Camera(){
  parameters = cv::aruco::DetectorParameters::create();
  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  params = cv::aruco::DetectorParameters::create();
  filename = "camera_info/head_camera.yaml";
  readOk = readCameraParameters(filename, cameraMatrix, distCoeffs);
}

// Draw x,y,z axis and corners of the detected ArUcO
void Autonomie::DessinAxis(){
  cv::aruco::drawAxis(cv_ptr->image, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
  cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
}

// Move the wifibot
void Autonomie::Move(float linear, float angular){
  // Wheels speed
  msg_cmd_vel.linear.x = linear;

  // Difference between left and right wheels speed
  msg_cmd_vel.angular.z = angular;
}

void Autonomie::Get_ArUcO_Position(){
  cv::aruco::estimatePoseSingleMarkers(corners, 0.17, cameraMatrix, distCoeffs, rvecs, tvecs);
  rvec = rvecs[rvecs.size()-1];
  tvec = tvecs[tvecs.size()-1]; 
}

void Autonomie::Course_Recording(){
  vel_t cvel;
  cvel.lx = msg_cmd_vel.linear.x;
  cvel.az = msg_cmd_vel.angular.z;
  tabvel.push_back(cvel);
}

void Autonomie::Course_Redo(){
  if(!redoStart){
    redoStart=true;
    redoTime = ros::Time::now();
  }
  while(compt<tabvel.size()-1){
    Move(tabvel.at(compt).lx, tabvel.at(compt).az);
    //std::cout << tabvel.at(compt).temps << std::endl;
    while(ros::Time::now() - redoTime < tabvel.at(compt+1).temps - tabvel.at(compt).temps){
    }
    cmd_vel_pub.publish(msg_cmd_vel);
    redoTime = ros::Time::now();
    compt++;
  }
  current=-1;
  compt=0;
}

void Autonomie::Course_Run(){
  if(tvec[2] > 0.25){
    // Moving forward
    LinearSpeed = 0.1*tvec[2];
    AngularSpeed = (-tvec[0]) + 0.15/rvec[1];
    Move(LinearSpeed, AngularSpeed);
  }
  else{
    // Moving backward
    LinearSpeed = -0.02/tvec[2];
    AngularSpeed = 0.3*(-tvec[0]);
    Move(LinearSpeed, AngularSpeed);
  }
}

void Autonomie::Update_GUI(){
  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);
}

void Autonomie::FollowAuto(){
  // Settings camera and detecting ArUcO
  if(readOk){
    cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);

    // if at least one marker detected
    if (ids.size() > 0){
      // We only take in account the first ones
      current = ids.at(0);
      Get_ArUcO_Position();
      if(current==0){
        Course_Run();
        Course_Recording();
        tabvel.at(tabvel.size()-1).temps = ros::Time::now();
        std::cout << tabvel.at(tabvel.size()-1).temps << std::endl;
      }
    }
    
    cmd_vel_pub.publish(msg_cmd_vel);
    
    ResetSpeed();
  }
  // Output modified video stream 
  Update_GUI();
    
}

void Autonomie::ResetSpeed(){
  Move(0,0);
}

void Autonomie::affichertab(){
  std::cout<<""<<std::endl;
  for (long unsigned int i = 0; i < tabvel.size(); ++i)
  {
    std::cout<<tabvel.at(i).lx<<std::endl;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  //doTime = ros::Time::now();
  Autonomie autonomie;
  
  ros::Rate r(100);
  while (ros::ok()) {
    if(autonomie.current == 1){
      autonomie.Course_Redo();
    }
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}