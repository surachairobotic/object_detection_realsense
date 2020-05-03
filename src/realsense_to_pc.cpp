#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>

#include "datmo/cConvert3D.h"

/*sensor_msgs::CameraInfo msg_cam_info;
sensor_msgs::Image msg_depth, msg_col;
sensor_msgs::PointCloud2 msg_pc_org*/

cv_bridge::CvImagePtr p_img_col;

sensor_msgs::CameraInfo msg_cam_info;
sensor_msgs::Image msg_col, msg_depth;
sensor_msgs::PointCloud2 pc;

bool b_cam_info = false, b_col = false, b_depth = false;


void cb_cam_info2(const sensor_msgs::CameraInfo& msg){
  msg_cam_info = msg;
  b_cam_info = true;
}
void cb_depth2(const sensor_msgs::Image& msg){
  msg_depth = msg;
  b_depth = true;
}
void cb_col2(const sensor_msgs::Image& msg){
  msg_col = msg;
  b_col = true;
}


int main(int argc, char **argv)
{
  // Initialize node and nodehandles
  ROS_INFO("start");
  ros::init(argc, argv, "datmo_realsense");
  ros::NodeHandle n;
    
  ros::Subscriber cam_info_sub = n.subscribe("/camera/aligned_depth_to_color/camera_info", 10, cb_cam_info2)
    , depth_sub = n.subscribe("/camera/aligned_depth_to_color/image_raw", 10, cb_depth2)
    , col_sub = n.subscribe("/camera/color/image_raw", 10, cb_col2);

  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("/datmo/point_cloud", 10);

  //while(b_cam_info);
  //cam_info_sub.shutdown();

  while (ros::ok())
  {
    ros::spinOnce();
    if( b_col ){
      
      b_col = false;
      p_img_col = cv_bridge::toCvCopy(msg_col, sensor_msgs::image_encodings::BGR8);
      cv::Mat img;
      cv::resize(p_img_col->image, img
        , cv::Size( p_img_col->image.cols/2, p_img_col->image.rows/2 ));
      cv::imshow("col", img);
      

        if( !b_depth ){
          ;//ROS_WARN("no depth");
        }
        else{
          ROS_INFO("start process");
          pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
          cConvert3D::convert_pc(msg_depth, msg_cam_info, msg_col, cloud_rgb);
          b_depth = false;
          pcl::toROSMsg(cloud_rgb, pc);
	  pc.header.frame_id = "world";
          pc.header.stamp = ros::Time::now();
          pub.publish(pc);
        }
    }
  }
  ROS_INFO("stop");
  return 0;
}
