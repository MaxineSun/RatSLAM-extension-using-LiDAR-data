#include "utils/utils.h"

#include <boost/property_tree/ini_parser.hpp>

#include <ros/ros.h>

#include "ratslam/ground_truth.h"
#include <nav_msgs/Odometry.h>
#include "graphics/ground_truth_scene.h"
#include <tf/transform_broadcaster.h>


#ifdef HAVE_IRRLICHT
#include "graphics/ground_truth_scene.h"
ratslam::GroundTruthScene *gts;
bool use_graphics;
#endif

using namespace ratslam;



void odo_callback(nav_msgs::OdometryConstPtr odo, ratslam::GroundTruth *gt)
{

  gt->create_pose(odo->pose.pose.position.x, odo->pose.pose.position.y, odo->pose.pose.orientation.z);

#ifdef HAVE_IRRLICHT
  if (use_graphics)
  {
    gts->update_scene();
    gts->draw_all();
  }
#endif
}


int main(int argc, char * argv[])
{

  if (argc < 2)
  {
    ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
    exit(-1);
  }
  std::string topic_root = "";
  boost::property_tree::ptree settings, general_settings, ratslam_settings;
  read_ini(argv[1], settings);

  get_setting_child(ratslam_settings, settings, "ratslam", true);
  get_setting_child(general_settings, settings, "general", true);
  get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string)"");

  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "RatSLAMGroundTruth");
  }
  ros::NodeHandle node;

  ratslam::GroundTruth * gt = new ratslam::GroundTruth(ratslam_settings);


  ros::Subscriber sub_odometry = node.subscribe<nav_msgs::Odometry>(topic_root + "/odom", 0, boost::bind(odo_callback, _1, gt), ros::VoidConstPtr(),
                                                                    ros::TransportHints().tcpNoDelay());


#ifdef HAVE_IRRLICHT
  boost::property_tree::ptree draw_settings;
  get_setting_child(draw_settings, settings, "draw", true);
  get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
  if (use_graphics)
  {
    gts = new ratslam::GroundTruthScene(draw_settings, gt);
  }
#endif

  ros::spin();

  return 0;
}