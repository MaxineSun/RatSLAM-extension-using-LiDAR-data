// class Scan
// {
// public:
//     Scan() 
//     {
//         pub_vt = node.advertise<ratslam_ros::ViewTemplate>(topic_root + "/LocalView/Template", 0);
//         sub = node.subscribe("scan", 0, &Scan::scan_callback, this);
//     }
//     void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
//     {
//       //ROS_DEBUG_STREAM("LV:image_callback{" << ros::Time::now() << "} seq=" << image->header.seq);

//       static ratslam_ros::ViewTemplate vt_output;
      
//       //lv->on_scan(&scan_in->ranges[0], (image->encoding == "bgr8" ? false : true), image->width, image->height);
//       if(!listener_.waitForTransform(
//                     scan_in->header.frame_id,
//                     "/odom",
//                     scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
//                     ros::Duration(1.0))){
//         return;
//       }
//       sensor_msgs::PointCloud2 cloud;
//       projector_.transformLaserScanToPointCloud("odom",*scan_in, cloud,listener_);
//       lv->on_scan(cloud);
//       vt_output.header.stamp = ros::Time::now();
//       vt_output.header.seq++;
//       vt_output.current_id = lv->get_current_vt();
//       vt_output.relative_rad = lv->get_relative_rad();

//       pub_vt.publish(vt_output);

//     #ifdef HAVE_IRRLICHT
//       if (use_graphics)
//       {
//         lvs->draw_all();
//       }
//     #endif
//     }
// protected:
//     ros::Publisher pub_vt;
//     ros::Subscriber sub;
//     ros::NodeHandle node;
//     laser_geometry::LaserProjection projector_;
//     tf::TransformListener listener_;
  
// };

// int main(int argc, char * argv[])
// {
//   ROS_INFO_STREAM(argv[0] << " - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
//   ROS_INFO_STREAM("RatSLAM algorithm by Michael Milford and Gordon Wyeth");
//   ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");

//   if (argc < 2)
//   {
//     ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
//     exit(-1);
//   }
//   std::string topic_root = "";

//   boost::property_tree::ptree settings, ratslam_settings, general_settings;
//   read_ini(argv[1], settings);

//   get_setting_child(general_settings, settings, "general", true);
//   get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string)"");
//   get_setting_child(ratslam_settings, settings, "ratslam", true);
//   lv = new ratslam::LocalViewMatch(ratslam_settings);

//   if (!ros::isInitialized())
//   {
//     ros::init(argc, argv, "RatSLAMViewTemplate");
//   }
//   Scan myscan;


// #ifdef HAVE_IRRLICHT
//     boost::property_tree::ptree draw_settings;
//     get_setting_child(draw_settings, settings, "draw", true);
//     get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
//     if (use_graphics)
//       lvs = new ratslam::LocalViewScene(draw_settings, lv);
// #endif

//   ros::spin();

//   return 0;
// }
