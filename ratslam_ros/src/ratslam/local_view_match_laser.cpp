/*
 * openRatSLAM
 *
 * utils - General purpose utility helper functions mainly for angles and readings settings
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "local_view_match_laser.h"
#include "../utils/utils.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <iostream>
#include <iomanip>
using namespace std;
#include <boost/foreach.hpp>
#include <algorithm>

#include <stdio.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
namespace ratslam
{



LocalViewMatch::LocalViewMatch(ptree settings)
{
  get_setting_from_ptree(VT_MIN_PATCH_NORMALISATION_STD, settings, "vt_min_patch_normalisation_std", (double)0);
  get_setting_from_ptree(VT_PATCH_NORMALISATION, settings, "vt_patch_normalise", 0);
  get_setting_from_ptree(VT_NORMALISATION, settings, "vt_normalisation", (double) 0);
  get_setting_from_ptree(VT_SHIFT_MATCH, settings, "vt_shift_match", 25);
  get_setting_from_ptree(VT_STEP_MATCH, settings, "vt_step_match", 5);
  get_setting_from_ptree(VT_PANORAMIC, settings, "vt_panoramic", 0);
 
  get_setting_from_ptree(VT_MATCH_THRESHOLD, settings, "vt_match_threshold", 0.03);
  get_setting_from_ptree(TEMPLATE_X_SIZE, settings, "template_x_size", 1);
  get_setting_from_ptree(TEMPLATE_Y_SIZE, settings, "template_y_size", 1);
  get_setting_from_ptree(IMAGE_VT_X_RANGE_MIN, settings, "image_crop_x_min", 0);
  get_setting_from_ptree(IMAGE_VT_X_RANGE_MAX, settings, "image_crop_x_max", -1);
  get_setting_from_ptree(IMAGE_VT_Y_RANGE_MIN, settings, "image_crop_y_min", 0);
  get_setting_from_ptree(IMAGE_VT_Y_RANGE_MAX, settings, "image_crop_y_max", -1);
  get_setting_from_ptree(LASER_AVG, settings, "laser_avg", 0);
  get_setting_from_ptree(LASER_VG, settings, "laser_vg", 0);
  get_setting_from_ptree(LASER_TEMPLATE_SIZE, settings, "laser_template_size", 1);
  get_setting_from_ptree(VT_MATCH_THRESHOLD_LASER, settings, "vt_match_threshold_laser", 0.03);
  get_setting_from_ptree(VT_PANORAMIC_LASER, settings, "vt_panoramic_laser", 0);

  TEMPLATE_SIZE = TEMPLATE_X_SIZE * TEMPLATE_Y_SIZE;

  templates.reserve(10000);

  current_view.resize(TEMPLATE_SIZE);
  current_view_laser.resize(LASER_TEMPLATE_SIZE);
  //current_view_vg.reset();
  current_vt = 0;
  prev_vt = 0;
}


LocalViewMatch::~LocalViewMatch()
{

}

void LocalViewMatch::on_scan(const float *ranges, unsigned int laser_size, float range_max)
{
  if(ranges == NULL)
    return;
  LASER_SIZE = laser_size;
  LASER_RANGE_MAX = (double)(range_max);
  this->ranges = ranges;
  convert_view_to_view_template_avg();
}

void LocalViewMatch::on_image(const unsigned char *view_rgb, bool greyscale, unsigned int image_width, unsigned int image_height)
  {
    if (view_rgb == NULL)
      return;
    
    IMAGE_WIDTH = image_width;
    IMAGE_HEIGHT = image_height;

    if (IMAGE_VT_X_RANGE_MAX == -1)
      IMAGE_VT_X_RANGE_MAX = IMAGE_WIDTH;
    if (IMAGE_VT_Y_RANGE_MAX == -1)
      IMAGE_VT_Y_RANGE_MAX = IMAGE_HEIGHT;

    this->view_rgb = view_rgb;
    this->greyscale = greyscale;

    convert_view_to_view_template(greyscale);
    //current_vt is an int
    prev_vt = get_current_vt();
    unsigned int vt_match_id;
    unsigned int vt_match_id_laser;
    
    //vt_error is a double 
    compare(vt_error, vt_match_id);
    if(vt_error < VT_MATCH_THRESHOLD){
      compare_fusion(vt_error_laser, vt_match_id_laser, vt_match_id);
      if (vt_error_laser <= VT_MATCH_THRESHOLD_LASER)
      {
        
          set_current_vt((int)vt_match_id);
          cout << "VTM[" << setw(4) << get_current_vt() << "] " << endl;
          cout << "Cam Error: " << vt_error << " Laser Error: " << vt_error_laser << endl;
          cout << "Cam VT: " << vt_match_id << " Laser VT: " << vt_match_id_laser << endl;
          cout.flush();
        
      }else
      {
        vt_relative_rad = 0;
        set_current_vt(create_template()); //create_template() returns an int, i.e., templates.size() - 1
        cout << "VTN[" << setw(4) << get_current_vt() << "] " << endl;
        cout.flush();
      } 
    }
    else
    {
      vt_relative_rad = 0;
      set_current_vt(create_template()); //create_template() returns an int, i.e., templates.size() - 1
      cout << "VTN[" << setw(4) << get_current_vt() << "] " << endl;
      cout.flush();
    }  
    

  }

void LocalViewMatch::convert_view_to_view_template_avg()
{
  int data_next = 0;
  int block_size = LASER_SIZE / LASER_TEMPLATE_SIZE;
  int pos;
  
  for (unsigned int i = 0; i < current_view_laser.size(); i++)
    current_view_laser[i] = 0;
  // cout << "Template: [";
  // for (std::vector<double>::const_iterator i = current_view_laser.begin(); i != current_view_laser.end(); ++i)
  //   cout << *i << ", ";
  // cout << "]" << endl;
  // cout.flush();
  
  for (int block_start = 0, block_count = 0; block_count < LASER_TEMPLATE_SIZE; block_start += block_size, block_count++)
  {
    for (int x = block_start; x < (block_start + block_size); x++)
    {
      if(isinf(ranges[x])){
        current_view_laser[data_next] += LASER_RANGE_MAX;
        // if(block_count < 3)
        //   cout << "Added value: " << LASER_RANGE_MAX << endl;
      } else {
        current_view_laser[data_next] += (double)(ranges[x]);
        // if(block_count < 3)
        //   cout << "Added value: " << (double)(ranges[x]) << endl;
      }
      
    }
    // cout << "Block sum:" << current_view_laser[data_next] << endl;
    // cout.flush();
    current_view_laser[data_next] /= LASER_RANGE_MAX;
    current_view_laser[data_next] /= block_size;
    data_next++;
  }
  double sum = 0;

  // find the mean of the data (after normalisation)
  for (int i = 0; i < current_view_laser.size(); i++)
    sum += current_view_laser[i];

  current_mean_laser = sum/current_view_laser.size();
  // cout << "Template: [";
  // for (unsigned int i = 0; i < current_view_laser.size(); i++)
  //   cout << i << ": " << current_view_laser[i] << ", ";
  // cout << "]" << endl;
  // cout.flush();
}  

void LocalViewMatch::compare_avg(double &vt_err_laser, unsigned int &vt_match_id_laser){
  if (templates.size() == 0)
  {
    vt_err_laser = DBL_MAX;
    vt_error_laser = vt_err_laser;
    return;
  }

  double *data = &current_view_laser[0];
  double mindiff, cdiff;
  mindiff = DBL_MAX;

  vt_err_laser = DBL_MAX;
  int min_template = 0;

  double *template_ptr;
  double *column_ptr;
  double *template_start_ptr;
  double *column_start_ptr;
  double *column_end_ptr;
  VisualTemplate vt;
  int min_offset;

  int offset;
  double epsilon = 0.005;

  BOOST_FOREACH(vt, templates)
  {

    if (abs(current_mean_laser - vt.mean_laser) > VT_MATCH_THRESHOLD_LASER + epsilon)
      continue;

    // for each vt try matching the view at different offsets
    // try to fast break based on error already great than previous errors
    // handles 2d images shifting only in the x direction
    // note I haven't tested on a 1d yet.
    for (offset = 0; offset < LASER_TEMPLATE_SIZE; offset += VT_STEP_MATCH)
    {
      cdiff = 0;
      template_start_ptr = &vt.data_laser[0] + offset;
      column_start_ptr = &data[0];
      column_end_ptr = &data[0] + LASER_TEMPLATE_SIZE - offset;
      

      // do from offset to end
      
      for (column_ptr = column_start_ptr, template_ptr = template_start_ptr; column_ptr < column_end_ptr; column_ptr++, template_ptr++)
      {
        cdiff += abs(*column_ptr - *template_ptr);
        // fast breaks
        if (cdiff > mindiff)
          break;
      }

      
      
      //e.g. offset = 10, LASER_TEMPLATE_SIZE = 60, template_start = vt.data[10], column_start = data[0], column_end = data[50]
      //column vs template = data[0] vs vt.data[10] to data[49] vs vt.data[59]

      // do from start to offset 
      // i.e., the elements that weren't compared by the previous for loop
      template_start_ptr = &vt.data[0];
      column_start_ptr = &data[0] + LASER_TEMPLATE_SIZE - offset;
      column_end_ptr = &data[0] + LASER_TEMPLATE_SIZE;
      for (column_ptr = column_start_ptr, template_ptr = template_start_ptr; column_ptr < column_end_ptr; column_ptr++, template_ptr++)
      {
        cdiff += abs(*column_ptr - *template_ptr);
        // fast breaks
        if (cdiff > mindiff)
          break;
      }
      //e.g. offset = 10, LASER_TEMPLATE_SIZE = 60, template_start = vt.data[0], column_start = data[50], column_end = data[60]
      //column vs template = data[50] vs vt.data[0] to data[59] vs vt.data[9]

      if (cdiff < mindiff)
      {
        mindiff = cdiff;
        min_template = vt.id;
        min_offset = offset;
      }
        
    }
      

  }

  vt_relative_rad_laser = (double) min_offset/LASER_TEMPLATE_SIZE * 2.0 * M_PI;
  if (vt_relative_rad > M_PI)
    vt_relative_rad = vt_relative_rad - 2.0 * M_PI;
  vt_err_laser = mindiff / (double) LASER_TEMPLATE_SIZE;
  vt_match_id_laser = min_template;

  vt_error_laser = vt_err_laser;
  cout << "Error: " << vt_error_laser << ", relative rot: " << min_offset << endl;
  cout.flush();
} 

void LocalViewMatch::compare_fusion(double &vt_err_laser, unsigned int &vt_match_id_laser, unsigned int vt_matched_camera){

  double *data = &current_view_laser[0];
  double mindiff, cdiff;
  mindiff = DBL_MAX;

  vt_err_laser = DBL_MAX;
  int min_template = 0;

  double *template_ptr;
  double *column_ptr;
  double *template_start_ptr;
  double *column_start_ptr;
  double *column_end_ptr;
  VisualTemplate vt;
  int min_offset;

  int offset;
  double epsilon = 0.005;
  int eval_range = 5;
  int lim_sup = vt_matched_camera + eval_range;
  int lim_inf = vt_matched_camera - eval_range;

  if(templates.size() < lim_sup){ lim_sup = templates.size();}
  if(0 > lim_inf){ lim_inf = 0;}
  for(int ids = lim_inf; ids < lim_sup; ids++)
  {
    vt = templates[ids];
    if (abs(current_mean_laser - vt.mean_laser) > VT_MATCH_THRESHOLD_LASER + epsilon)
      continue;

    // for each vt try matching the view at different offsets
    // try to fast break based on error already great than previous errors
    // handles 2d images shifting only in the x direction
    // note I haven't tested on a 1d yet.
    for (offset = 0; offset < LASER_TEMPLATE_SIZE; offset += VT_STEP_MATCH)
    {
      cdiff = 0;
      template_start_ptr = &vt.data_laser[0] + offset;
      column_start_ptr = &data[0];
      column_end_ptr = &data[0] + LASER_TEMPLATE_SIZE - offset;
      

      // do from offset to end
      
      for (column_ptr = column_start_ptr, template_ptr = template_start_ptr; column_ptr < column_end_ptr; column_ptr++, template_ptr++)
      {
        cdiff += abs(*column_ptr - *template_ptr);
        // fast breaks
        if (cdiff > mindiff)
          break;
      }

      
      
      //e.g. offset = 10, LASER_TEMPLATE_SIZE = 60, template_start = vt.data[10], column_start = data[0], column_end = data[50]
      //column vs template = data[0] vs vt.data[10] to data[49] vs vt.data[59]

      // do from start to offset 
      // i.e., the elements that weren't compared by the previous for loop
      template_start_ptr = &vt.data[0];
      column_start_ptr = &data[0] + LASER_TEMPLATE_SIZE - offset;
      column_end_ptr = &data[0] + LASER_TEMPLATE_SIZE;
      for (column_ptr = column_start_ptr, template_ptr = template_start_ptr; column_ptr < column_end_ptr; column_ptr++, template_ptr++)
      {
        cdiff += abs(*column_ptr - *template_ptr);
        // fast breaks
        if (cdiff > mindiff)
          break;
      }
      //e.g. offset = 10, LASER_TEMPLATE_SIZE = 60, template_start = vt.data[0], column_start = data[50], column_end = data[60]
      //column vs template = data[50] vs vt.data[0] to data[59] vs vt.data[9]

      if (cdiff < mindiff)
      {
        mindiff = cdiff;
        min_template = vt.id;
        min_offset = offset;
      }
        
    }
      

  }

  vt_relative_rad_laser = (double) min_offset/LASER_TEMPLATE_SIZE * 2.0 * M_PI;
  if (vt_relative_rad > M_PI)
    vt_relative_rad = vt_relative_rad - 2.0 * M_PI;
  vt_err_laser = mindiff / (double) LASER_TEMPLATE_SIZE;
  vt_match_id_laser = min_template;

  vt_error_laser = vt_err_laser;
  //cout << "Error: " << vt_error_laser << ", relative rot: " << min_offset << endl;
  //cout.flush();
} 
void LocalViewMatch::on_scan(sensor_msgs::PointCloud2 view_cloud)
{
  // if (view_rgb == NULL)
  //   return;

  this->view_cloud = view_cloud;
  

  convert_view_to_view_template_vg();
  prev_vt = get_current_vt();
  unsigned int vt_match_id;
  // compare(vt_error, vt_match_id);
  // if (vt_error <= VT_MATCH_THRESHOLD)
  // {
  //   set_current_vt((int)vt_match_id);
  //   cout << "VTM[" << setw(4) << get_current_vt() << "] " << endl;
  //   cout.flush();
  // }
  // else
  // {
  //   vt_relative_rad = 0;
  //   set_current_vt(create_template());
  //   cout << "VTN[" << setw(4) << get_current_vt() << "] " << endl;
  //   cout.flush();
  // }

}

void LocalViewMatch::convert_view_to_view_template_vg()
{
  //code taken from http://wiki.ros.org/pcl/Tutorials
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(view_cloud, *cloud);
  // cout << "PointCloud before filtering: " << cloud->width * cloud->height 
  //     << " data points (" << pcl::getFieldsList (*cloud) << ")." << endl;
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);
  pcl::fromPCLPointCloud2(cloud_filtered,*current_view_vg);
  // cout << "PointCloud after filtering: " << cloud_filtered.width * cloud_filtered.height 
  //     << " data points (" << pcl::getFieldsList (cloud_filtered) << ")." << endl;
  // for(int i = 0; i < temp_cloud->points.size(); i++){
  //   cout << setw(3) << i << " x: " << temp_cloud->points[i].x << " y: " << temp_cloud->points[i].y 
  //     << " z: " << temp_cloud->points[i].z << endl;
  // }
  // cout.flush();
  // Convert to ROS data type
  // sensor_msgs::PointCloud2 output;
  // pcl_conversions::fromPCL(cloud_filtered, output);


}

int LocalViewMatch::create_template_vg()
{
  templates.resize(templates.size() + 1);
  VisualTemplate * new_template = &(*(templates.end() - 1));

  new_template->id = templates.size() - 1;
  double * data_ptr = &current_view[0];
  new_template->data.reserve(LASER_TEMPLATE_SIZE);
  for (int i = 0; i < LASER_TEMPLATE_SIZE; i++)
    new_template->data.push_back(*(data_ptr++));

  new_template->mean = current_mean;

  return templates.size() - 1;
}

  //clip x and y so that their values are between 0 and the TEMPLATE_SIZE
  //is used for patch normalisation
  void LocalViewMatch::clip_view_x_y(int &x, int &y)
  {
    if (x < 0)
      x = 0;
    else if (x > TEMPLATE_X_SIZE - 1)
      x = TEMPLATE_X_SIZE - 1;

    if (y < 0)
      y = 0;
    else if (y > TEMPLATE_Y_SIZE - 1)
      y = TEMPLATE_Y_SIZE - 1;

  }

  void LocalViewMatch::convert_view_to_view_template(bool grayscale)
  {
    int data_next = 0;
    int sub_range_x = IMAGE_VT_X_RANGE_MAX - IMAGE_VT_X_RANGE_MIN;
    int sub_range_y = IMAGE_VT_Y_RANGE_MAX - IMAGE_VT_Y_RANGE_MIN;
    int x_block_size = sub_range_x / TEMPLATE_X_SIZE;
    int y_block_size = sub_range_y / TEMPLATE_Y_SIZE;
    int pos;

    for (unsigned int i; i < current_view.size(); i++)
      current_view[i] = 0;

    if (grayscale)
    {
      for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block +=
          y_block_size, y_block_count++)
      {
        for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block +=
            x_block_size, x_block_count++)
        {
          for (int x = x_block; x < (x_block + x_block_size); x++)
          {
            for (int y = y_block; y < (y_block + y_block_size); y++)
            {
              pos = (x + y * IMAGE_WIDTH);
              current_view[data_next] += (double)(view_rgb[pos]);
            }
          }
          current_view[data_next] /= (255.0); //normalize so the value in the vector is between 0.0 and 1.0
          current_view[data_next] /= (x_block_size * y_block_size); //average among the block elements
          data_next++; //go to next element of current_view, i.e., the next block in the image
        } 
      }
    }
    else //if color image - same logic as for grayscale image
    {
      for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block +=
          y_block_size, y_block_count++)
      {
        for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block +=
            x_block_size, x_block_count++)
        {
          for (int x = x_block; x < (x_block + x_block_size); x++)
          {
            for (int y = y_block; y < (y_block + y_block_size); y++)
            {
              pos = (x + y * IMAGE_WIDTH) * 3;
              current_view[data_next] += ((double)(view_rgb[pos]) + (double)(view_rgb[pos + 1])
                  + (double)(view_rgb[pos + 2]));
            }
          }
          current_view[data_next] /= (255.0 * 3.0);
          current_view[data_next] /= (x_block_size * y_block_size);

          data_next++;
        }
      }
    }

    if (VT_NORMALISATION > 0)
    {
      double avg_value = 0;

      for (unsigned int i = 0; i < current_view.size(); i++)
      {
        avg_value += current_view[i];
      }

      avg_value /= current_view.size();

      for (unsigned int i = 0; i < current_view.size(); i++)
      {
        current_view[i] = std::max(0.0, std::min(current_view[i] * VT_NORMALISATION / avg_value, 1.0));
      }
    }

    // now do patch normalisation
    // +- patch size on the pixel, ie 4 will give a 9x9
    if (VT_PATCH_NORMALISATION > 0)
    {
      int patch_size = VT_PATCH_NORMALISATION;
      int patch_total = (patch_size * 2 + 1) * (patch_size * 2 + 1);
      double patch_sum;
      double patch_mean;
      double patch_std;
      int patch_x_clip;
      int patch_y_clip;

      // first make a copy of the view
      std::vector<double> current_view_copy;
      current_view_copy.resize(current_view.size());
      for (unsigned int i = 0; i < current_view.size(); i++)
        current_view_copy[i] = current_view[i];

      // this code could be significantly optimimised ....
      for (int x = 0; x < TEMPLATE_X_SIZE; x++)
      {
        for (int y = 0; y < TEMPLATE_Y_SIZE; y++)
        {
          patch_sum = 0;
          //this for loop is to compute the mean of the patch
          for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)
          {
            for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)
            {
              patch_x_clip = patch_x;
              patch_y_clip = patch_y;
              clip_view_x_y(patch_x_clip, patch_y_clip);

              patch_sum += current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE];
            }
          }
          patch_mean = patch_sum / patch_total;
          //this for loop is to compute the std of the patch
          patch_sum = 0;
          for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)
          {
            for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)
            {
              patch_x_clip = patch_x;
              patch_y_clip = patch_y;
              clip_view_x_y(patch_x_clip, patch_y_clip);

              patch_sum += ((current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE] - patch_mean)
                  * (current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE] - patch_mean));
            }
          }

          patch_std = sqrt(patch_sum / patch_total);

          if (patch_std < VT_MIN_PATCH_NORMALISATION_STD)
            current_view[x + y * TEMPLATE_X_SIZE] = 0.5;
          else {
            current_view[x + y * TEMPLATE_X_SIZE] = max((double) 0, min(1.0, (((current_view_copy[x + y * TEMPLATE_X_SIZE] - patch_mean) / patch_std) + 3.0)/6.0 ));
          }
        }
      }
    }//end of the patch normalisation

    double sum = 0;

    // find the mean of the data (after normalisation)
    for (int i = 0; i < current_view.size(); i++)
      sum += current_view[i];

    current_mean = sum/current_view.size();

  } // end of conversion from view to view template (including patch normalisation)

  // create and add a visual template to the collection
  int LocalViewMatch::create_template()
  {
    templates.resize(templates.size() + 1);
    VisualTemplate * new_template = &(*(templates.end() - 1));

    new_template->id = templates.size() - 1;
    double * data_ptr = &current_view[0];
    double * data_ptr_laser = &current_view_laser[0];
    new_template->data.reserve(TEMPLATE_SIZE);
    for (int i = 0; i < TEMPLATE_SIZE; i++)
      new_template->data.push_back(*(data_ptr++));

    new_template->mean = current_mean;
    
    new_template->data_laser.reserve(LASER_TEMPLATE_SIZE);
    for (int i = 0; i < LASER_TEMPLATE_SIZE; i++)
      new_template->data_laser.push_back(*(data_ptr_laser++));

    new_template->mean_laser = current_mean_laser;

    return templates.size() - 1;
  }

  // compare a visual template to all the stored templates, allowing for 
  // slen pixel shifts in each direction
  // returns the matching template and the MSE
  void LocalViewMatch::compare(double &vt_err, unsigned int &vt_match_id)
  {
    if (templates.size() == 0)
    {
      vt_err = DBL_MAX;
      vt_error = vt_err;
      return;
    }

    double *data = &current_view[0];
    double mindiff, cdiff;
    mindiff = DBL_MAX;

    vt_err = DBL_MAX;
    int min_template = 0;

    double *template_ptr;
    double *column_ptr;
    double *template_row_ptr;
    double *column_row_ptr;
    double *template_start_ptr;
    double *column_start_ptr;
    int row_size;
    int sub_row_size;
    double *column_end_ptr;
    VisualTemplate vt;
    int min_offset;

    int offset;
    double epsilon = 0.005;

    if (VT_PANORAMIC)
    {

    BOOST_FOREACH(vt, templates)
    {

    if (abs(current_mean - vt.mean) > VT_MATCH_THRESHOLD + epsilon)
      continue;

    // for each vt try matching the view at different offsets
    // try to fast break based on error already great than previous errors
    // handles 2d images shifting only in the x direction
    // note I haven't tested on a 1d yet.
    for (offset = 0; offset < TEMPLATE_X_SIZE; offset += VT_STEP_MATCH)
    {
      cdiff = 0;
      template_start_ptr = &vt.data[0] + offset;
      column_start_ptr = &data[0];
      row_size = TEMPLATE_X_SIZE;
      column_end_ptr = &data[0] + TEMPLATE_SIZE - offset;
      sub_row_size = TEMPLATE_X_SIZE - offset;
      

      // do from offset to end
      for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
      {
        for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
        {
          cdiff += abs(*column_ptr - *template_ptr);
        }

        // fast breaks
        if (cdiff > mindiff)
          break;
      }
      //e.g. offset = 10, TEMPLATE_SIZE = 1200 (60x20), sub_row_size = 50, template_start = vt.data[10], column_start = data[0], column_end = data[1190]
      //column_row = data[0] -> data[60] -> data[120] ... -> data[1140] -> break
      //template_row = vt.data[10] -> vt.data[70] ... -> vt.data[1150] -> break
      //column vs template = data[start] vs vt.data[start+10] to data[start+49] vs vt.data[start+59]

      // do from start to offset
      template_start_ptr = &vt.data[0];
      column_start_ptr = &data[0] + TEMPLATE_X_SIZE - offset;
      row_size = TEMPLATE_X_SIZE;
      column_end_ptr = &data[0] + TEMPLATE_SIZE;
      sub_row_size = offset;
      for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
      {
        for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
        {
          cdiff += abs(*column_ptr - *template_ptr);
        }

        // fast breaks
        if (cdiff > mindiff)
          break;
      }
      //e.g. offset = 10, TEMPLATE_SIZE = 1200 (60x20), sub_row_size = 10, template_start = vt.data[0], column_start = data[50], column_end = data[1200]
      //column_row = data[50] -> data[110] -> data[170] ... -> data[1190] -> break
      //template_row = vt.data[0] -> vt.data[60] ... -> vt.data[1140] -> break
      //column vs template = data[start+50] vs vt.data[start] to data[start+59] vs vt.data[start+9]


      if (cdiff < mindiff)
      {
      mindiff = cdiff;
      min_template = vt.id;
      min_offset = offset;
      }
    }

    }

    vt_relative_rad = (double) min_offset/TEMPLATE_X_SIZE * 2.0 * M_PI;
    if (vt_relative_rad > M_PI)
      vt_relative_rad = vt_relative_rad - 2.0 * M_PI;
    vt_err = mindiff / (double) TEMPLATE_SIZE;
    vt_match_id = min_template;

    vt_error = vt_err;

    } else { //if not VT_PANORAMIC

    BOOST_FOREACH(vt, templates)
    {

    if (abs(current_mean - vt.mean) > VT_MATCH_THRESHOLD + epsilon)
      continue;

    // for each vt try matching the view at different offsets
    // try to fast break based on error already great than previous errors
    // handles 2d images shifting only in the x direction
    // note I haven't tested on a 1d yet.
    for (offset = 0; offset < VT_SHIFT_MATCH*2+1; offset += VT_STEP_MATCH)
    {
      cdiff = 0;
      template_start_ptr = &vt.data[0] + offset;
      column_start_ptr = &data[0] + VT_SHIFT_MATCH;
      row_size = TEMPLATE_X_SIZE;
      column_end_ptr = &data[0] + TEMPLATE_SIZE - VT_SHIFT_MATCH;
      sub_row_size = TEMPLATE_X_SIZE - 2*VT_SHIFT_MATCH;

      for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
      {
        for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
        {
          cdiff += abs(*column_ptr - *template_ptr);
        }

      // fast breaks
      if (cdiff > mindiff)
        break;
      }
      //e.g. offset = 8, TEMPLATE_SIZE = 1200 (60x20), VT_SHIFT_MATCH = 4, sub_row_size = 52, template_start = vt.data[8], column_start = data[4], column_end = data[1196]
      //column_row = data[4] -> data[64] -> data[124] ... -> data[1144] -> break
      //template_row = vt.data[8] -> vt.data[68] ... -> vt.data[1148] -> break
      //column vs template = data[start] vs vt.data[start+4] to data[start+51] vs vt.data[start+59] (for first row: start=4)
      //if offset = 0 then column vs template = data[start] vs vt.data[start-4] to data[start+51] vs vt.data[start+47] (for first row: start=4)

      if (cdiff < mindiff)
      {
      mindiff = cdiff;
      min_template = vt.id;
      min_offset = 0;
      }
    }

    }

    vt_relative_rad = 0;
    vt_err = mindiff / (double)(TEMPLATE_SIZE - 2 * VT_SHIFT_MATCH * TEMPLATE_Y_SIZE);
    vt_match_id = min_template;

    vt_error = vt_err;

    } // end of not VT_PANORAMIC
  }//end of compare
}
