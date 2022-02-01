#ifndef _GROUND_TRUTH_H_
#define _GROUND_TRUTH_H_
#define _USE_MATH_DEFINES
#include "math.h"

#include <stdio.h>
#include <vector>
#include <deque>

#include <iostream>

#include <boost/property_tree/ini_parser.hpp>
using boost::property_tree::ptree;

#include <boost/serialization/access.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/deque.hpp>

namespace ratslam
{


/*
 * The Experience structure describes
 * a node in the Experience_Map.
 */
struct TruePose
{
  int id; // its own id

  double x_m, y_m, th_rad;


  template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
      ar & id;
      ar & x_m & y_m & th_rad;
    }

};

class GroundTruthScene;

class GroundTruth
{

public:
  friend class GroundTruthScene;

  GroundTruth(ptree settings);
  ~GroundTruth();

  // create a new experience for a given position
  int create_pose(double x, double y, double q_z);

  TruePose *get_pose(int id)
  {
    return &poses[id];
  }

  int get_num_poses()
  {
    return poses.size();
  }

  int get_current_id()
  {
    return current_pose_id;
  }

  template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {

      ar & GT_INITIAL_DEG;

      ar & poses;

      ar & current_pose_id & prev_pose_id;

    }

private:
  friend class boost::serialization::access;

  GroundTruth()
  {
    ;
  }

  double GT_INITIAL_DEG;

  std::vector<TruePose> poses;

  int current_pose_id, prev_pose_id;

};

}

#endif // _EXPERIENCE_MAP_H_
