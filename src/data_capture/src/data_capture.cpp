#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h> 

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>

#include "ros/ros.h"
#include "std_msgs/String.h"


#include "common_func/move_ctrl.h"

#define IMAGE_ROOT_DIR        ("/mnt/zed_data/picture_zed/")
#define CLOUDPOINT_ROOT_DIR   ("/mnt/zed_data/pointclond_zed/")
#define MAX_TIME_INFO_LEN (100)
#define MAX_CMD_LEN (200)

using namespace std;

static int pic_index = 0;
static int cloud_index = 0;

static char cur_valid_picture_path[MAX_TIME_INFO_LEN] = {0};
static char cur_valid_cloud_path[MAX_TIME_INFO_LEN] = {0};
static bool sample_switch = false;

static ros::Publisher *status_publisher = NULL;

static int mkdir_new_data_folder(string *pre_folder)
{
    //get current time
    time_t now = time(0);
    tm *ltm = localtime(&now);
    char time_info[MAX_TIME_INFO_LEN] = {0};
    char cmd[MAX_CMD_LEN] = {0};

    sprintf(time_info, "%d-%d-%d-%d_%d_%d/", 1900 + ltm->tm_year, ltm->tm_mon, ltm->tm_mday, ltm->tm_hour ,ltm->tm_min, ltm->tm_sec);
    cout << "[timestamp]: " << time_info << endl;

    pre_folder->append(time_info);

    sprintf(cmd, "mkdir -p %s", pre_folder->c_str());

    printf("%s\r\n", pre_folder->c_str());

    if( (access( pre_folder->c_str(), F_OK )) != 0 )
    {
        system(cmd);
        cout << "[cmd]: " << cmd << endl;
    }

    return 0;
}


static int create_new_data_folder()
{
   string iamge_dir = IMAGE_ROOT_DIR;
   string cloud_dir = CLOUDPOINT_ROOT_DIR;

    pic_index = 0;
    cloud_index = 0;
    mkdir_new_data_folder(&iamge_dir);
    mkdir_new_data_folder(&cloud_dir);
    memset(cur_valid_picture_path, 0, sizeof(cur_valid_picture_path));
    memset(cur_valid_cloud_path, 0, sizeof(cur_valid_cloud_path));
    sprintf(cur_valid_picture_path, "%s", iamge_dir.c_str());
    sprintf(cur_valid_cloud_path, "%s", cloud_dir.c_str());

    cout << "new pic folder: " << cur_valid_picture_path <<endl;
    cout << "new clo folder: " << cur_valid_cloud_path <<endl;

    return 0;
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void image_callback(const sensor_msgs::ImageConstPtr &msg)
{
  //ROS_INFO("nick enter image_callback");

  pic_index++;
}


void pointcloud2_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  ROS_INFO("nick enter pointcloud2_callback");
  if(sample_switch == false)
  {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PCDWriter writer;

  pcl::fromROSMsg(*msg, cloud);


  char pcd_file_name[MAX_TIME_INFO_LEN] = {0};
  sprintf(pcd_file_name, "%scloud_pont_%d.pcd", cur_valid_cloud_path, cloud_index);

  cout << pcd_file_name << endl;

  writer.writeBinaryCompressed(pcd_file_name, cloud);

  cloud_index++;
}


void rc_info_callback(const std_msgs::UInt64::ConstPtr& msg)
{
  //ROS_INFO("nick enter rc_info_callback");
  static uint16_t last_work_status = PAUSE;
  static uint16_t cur_work_status = PAUSE;


  uint64_t recv_data = msg->data;
  uint16_t work_mode = recv_data & 0xFFFF;
  uint16_t work_status = (recv_data >> 16) & 0xFFFF;

  last_work_status = cur_work_status;
  cur_work_status = work_status;

  if((last_work_status == PAUSE) && (cur_work_status == PAUSE))   { sample_switch = false; }

  //start new folder
  if((last_work_status == PAUSE) && (cur_work_status == WORKING))
  {
    create_new_data_folder();
    sample_switch = false;
    //send begin sample flag to pwm ctrl
    std_msgs::UInt8 msg;
    msg.data = PROG_TASK_BEGIN;
    status_publisher->publish(msg);
  }

  if((last_work_status == WORKING) && (cur_work_status == WORKING))
  {
    //enable sample
    sample_switch = true;
  }

  if((last_work_status == WORKING) && (cur_work_status == PAUSE))
  {
      sample_switch = false;
      //send stop sample flag to pwm ctrl
      std_msgs::UInt8 msg;
      msg.data = PROG_TASK_END;
      status_publisher->publish(msg);
  }
}




int main(int argc, char **argv)
{
   time_t curtime;
   time(&curtime);
   string iamge_dir = IMAGE_ROOT_DIR;
   string cloud_dir = CLOUDPOINT_ROOT_DIR;

  /* check if the SSD has been mount to the /mnt */
  if( (access( IMAGE_ROOT_DIR, F_OK )) != 0 )
  {
      ROS_ERROR("%s dnesn't exist", IMAGE_ROOT_DIR);
      return -1;
  }

  if( (access( CLOUDPOINT_ROOT_DIR, F_OK )) != 0 )
  {
      ROS_ERROR("%s dnesn't exist", CLOUDPOINT_ROOT_DIR);
      return -1;
  }


  //----step1: create folder as timestamp
  //create_new_data_folder();




  ros::init(argc, argv, "listener");


  ros::NodeHandle image_handle;
  ros::NodeHandle point_handle;
  ros::NodeHandle rc_handle;
  ros::NodeHandle sample_status;  

  ros::Subscriber image_sub = image_handle.subscribe("/zed/zed_node/left/image_rect_color", 10, image_callback);
  ros::Subscriber pointcloud2_sub = point_handle.subscribe("/zed/zed_node/point_cloud/cloud_registered", 10, pointcloud2_callback);
  ros::Subscriber rc_info = rc_handle.subscribe(RC_CTRL_INFO, 100, rc_info_callback);

  ros::Publisher chatter_pub = sample_status.advertise<std_msgs::UInt8>(PROG_STATUS_TOPIC, 100);
  status_publisher = &chatter_pub;

  //wait PWM ready
  sleep(3);

  std_msgs::UInt8 msg;
  msg.data = PROG_TASK_READY;
  status_publisher->publish(msg);

  ros::spin();

  return 0;
}