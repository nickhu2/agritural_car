#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h> 

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#define IMAGE_ROOT_DIR        ("/mnt/zed_data/picture_zed/")
#define CLOUDPOINT_ROOT_DIR   ("/mnt/zed_data/pointclond_zed/")
#define MAX_TIME_INFO_LEN (100)
#define MAX_CMD_LEN (200)

using namespace std;

static int pic_index = 0;
static int cloud_index = 0;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void image_callback(const sensor_msgs::ImageConstPtr &msg)
{
  ROS_INFO("nick enter image_callback");

  pic_index++;
}


void pointcloud2_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  ROS_INFO("nick enter pointcloud2_callback");

  cloud_index++;
}

static int mkdir_new_data_folder(string *pre_folder)
{
    //get current time
    time_t now = time(0);
    tm *ltm = localtime(&now);
    char time_info[MAX_TIME_INFO_LEN] = {0};
    char cmd[MAX_CMD_LEN] = {0};

    sprintf(time_info, "%d-%d-%d-%d:%d:%d/", 1900 + ltm->tm_year, ltm->tm_mon, ltm->tm_mday, ltm->tm_hour ,ltm->tm_min, ltm->tm_sec);
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
  mkdir_new_data_folder(&iamge_dir);

  //----step2: excute picture saving node
  string pic_saver_cmd = "rosrun image_view image_saver \"_filename_format:=";
  pic_saver_cmd.append(iamge_dir);
  pic_saver_cmd.append("image_%06d.%s\" image:=/zed/zed_node/left/image_rect_color &");  //background
  cout << "[cmd]: " << pic_saver_cmd << endl;
  system(pic_saver_cmd.c_str());



  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::Subscriber image_sub = n.subscribe("/zed/zed_node/left/image_rect_color", 10, image_callback);
  ros::Subscriber pointcloud2_sub = n.subscribe("/zed/zed_node/point_cloud/cloud_registered", 10, pointcloud2_callback);


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}