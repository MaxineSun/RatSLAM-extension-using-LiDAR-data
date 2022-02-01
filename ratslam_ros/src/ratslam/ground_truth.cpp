#include "ground_truth.h"
#include "../utils/utils.h"

#include <queue>
#include <float.h>
#include <iostream>

using namespace std;

namespace ratslam
{

GroundTruth::GroundTruth(ptree settings)
{
  get_setting_from_ptree(GT_INITIAL_DEG, settings, "exp_initial_em_deg", 90.0);


  poses.reserve(10000);

  current_pose_id = 0;
  prev_pose_id = 0;

}

GroundTruth::~GroundTruth()
{
  poses.clear();
}

// create a new experience for a given position 
int GroundTruth::create_pose(double x, double y, double q_z)
{

  poses.resize(poses.size() + 1);
  TruePose * new_pose = &(*(poses.end() - 1));

  if (poses.size() == 0)
  {
    new_pose->x_m = 0;
    new_pose->y_m = 0;
    new_pose->th_rad = 0;
  }
  else
  {
    new_pose->x_m = x;
    new_pose->y_m = y;
    new_pose->th_rad = clip_rad_180(asin(q_z) * 2.0);
  }
  new_pose->id = poses.size() - 1;
  current_pose_id = poses.size() - 1;
  if(current_pose_id > 0)
    prev_pose_id = current_pose_id - 1;

  return poses.size() - 1;
}



} // namespace ratslam

