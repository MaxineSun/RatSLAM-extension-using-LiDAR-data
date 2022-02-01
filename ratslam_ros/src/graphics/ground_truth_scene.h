#ifndef GROUND_TRUTH_SCENE_HPP
#define GROUND_TRUTH_SCENE_HPP
#include "../ratslam/ground_truth.h"
#include "../utils/utils.h"

#include <irrlicht/irrlicht.h>

#define _USE_MATH_DEFINES
#include <cmath>

#include "path_node.h"

namespace ratslam
{

class GroundTruthScene
{
public:
  GroundTruthScene(ptree & settings, GroundTruth *in_gt) :
      gt_map_scene(NULL), gt_map_path(NULL), map(in_gt)
  {

    int width = 800;
    int height = 800;

    device = irr::createDevice(irr::video::EDT_OPENGL, irr::core::dimension2d<irr::u32>(width, height), 32, false,
                               false, false);

    device->setWindowCaption(L"openRatSLAM Ground Truth");
    driver = device->getVideoDriver();
    scene = device->getSceneManager();

    irr::scene::ICameraSceneNode * camera = scene->addCameraSceneNode(
        NULL, irr::core::vector3df((irr::f32)(0.5 * width), (irr::f32)(0.5 * height), -10),
        irr::core::vector3df((irr::f32)(0.5 * width), (irr::f32)(0.5 * height), 0));
    irr::core::matrix4 proj;
    proj.buildProjectionMatrixOrthoLH((irr::f32)width, (irr::f32)height, (irr::f32)0.01, (irr::f32)100);
    camera->setProjectionMatrix(proj, true);

    gt_map_scene = scene->createNewSceneManager(false);

    // the path as lines
    gt_map_path = new PathNode(gt_map_scene->getRootSceneNode());
    gt_map_path->setPrimitiveType(irr::scene::EPT_LINES);
    gt_map_path->getMaterial(0).EmissiveColor = irr::video::SColor(255, 255, 255, 255);
    gt_map_path->getMaterial(0).Thickness = 2.0f;

    // todo: use relative path here and if can't find the image then draw a simple robot shape box
    get_setting_from_ptree(media_path, settings, "media_path", (std::string)"");
    get_setting_from_ptree(image_file, settings, "image_file", (std::string)"");

    // add the irat texture
    irr::video::ITexture * irat_texture = gt_map_scene->getVideoDriver()->getTexture(
        (media_path + "/" + image_file).c_str());
    if (irat_texture == 0)
    {
      std::cout << "WARNING: Unable to load texture: " << (media_path + "/" + image_file) << std::endl;
    }

    // add a cube node and texture it with the irat
    if (irat_texture)
    {
    irat_node = gt_map_scene->addMeshSceneNode(
        gt_map_scene->getGeometryCreator()->createCubeMesh(
            irr::core::vector3df((irr::f32)(irat_texture->getSize().Width / 8.0),
                                 (irr::f32)(irat_texture->getSize().Height / 8.0), (irr::f32)0.1)),
        NULL);
    }
    else
    {
    irat_node = gt_map_scene->addMeshSceneNode(
        gt_map_scene->getGeometryCreator()->createCubeMesh(
            irr::core::vector3df((irr::f32)(535.0 / 8.0),
                                 (irr::f32)(289.0 / 8.0), (irr::f32)0.1)),
        NULL);
    }
    irat_node->getMaterial(0).EmissiveColor = irr::video::SColor(255, 255, 255, 255);
	if (irat_texture)
	{
		irat_node->getMaterial(0).setTexture(0, irat_texture);
	}
    irat_node->getMaterial(0).MaterialType = irr::video::EMT_TRANSPARENT_ALPHA_CHANNEL_REF;

    update_viewport(0, 0, width, height);

  }

  ~GroundTruthScene()
  {
    gt_map_scene->drop();
    delete gt_map_path;
  }

  void update_poses()
  {
    double x, y; //, facing;
    unsigned int i;

    min_x = DBL_MAX;
    max_x = -DBL_MAX;
    min_y = DBL_MAX;
    max_y = -DBL_MAX;

    for (i = 0; i < map->poses.size(); i++)
    {
      x = map->poses[i].x_m;
      y = map->poses[i].y_m;

      if (x < min_x)
      {
        min_x = x;
      }

      if (x > max_x)
      {
        max_x = x;
      }

      if (y < min_y)
      {
        min_y = y;
      }

      if (y > max_y)
      {
        max_y = y;
      }

    }

    // account for the size of the robot
    min_x = min_x - 0.1;
    min_y = min_y - 0.1;
    max_x = max_x + 0.1;
    max_y = max_y + 0.1;

  }

  void update_current_pose(int pose)
  {
    float x2d1, y2d1;
    for(unsigned int i = 0; i < map->poses.size(); i++){
      x2d1 = (irr::f32)((map->poses[i].x_m - min_x) / (max_x - min_x) * viewport_width);
      y2d1 = (irr::f32)((map->poses[i].y_m - min_y) / (max_y - min_y) * viewport_height);
      gt_map_path->addPoint(irr::core::vector3df((irr::f32)x2d1, (irr::f32)y2d1, (irr::f32)0));
    }
    x2d1 = (irr::f32)((map->poses[pose].x_m - min_x) / (max_x - min_x) * viewport_width);
    y2d1 = (irr::f32)((map->poses[pose].y_m - min_y) / (max_y - min_y) * viewport_height);
    
    irat_node->setPosition(irr::core::vector3df(x2d1, y2d1, 0));

    while (map->poses[pose].th_rad > M_PI)
    {
      map->poses[pose].th_rad -= 2 * M_PI;
    }

    while (map->poses[pose].th_rad < -M_PI)
    {
      map->poses[pose].th_rad += 2 * M_PI;
    }

	double agent_rad = map->poses[pose].th_rad;
    double facing_ratio = tan(agent_rad);

    if (agent_rad > 0 && agent_rad < M_PI_2)
    {
      irat_node->setRotation(
          irr::core::vector3df(0, 0, (irr::f32)(atan2(facing_ratio * viewport_height, viewport_width) * 180 / M_PI)));
    }
    else if (agent_rad > M_PI_2 && agent_rad < M_PI)
    {
      irat_node->setRotation(
          irr::core::vector3df(0, 0, (irr::f32)(atan2(-facing_ratio * viewport_height, -viewport_width) * 180 / M_PI)));
    }
    else if (agent_rad < 0 && agent_rad > -M_PI_2)
    {
      irat_node->setRotation(
          irr::core::vector3df(0, 0, (irr::f32)(atan2(facing_ratio * viewport_height, viewport_width) * 180 / M_PI)));
    }
    else if (agent_rad < -M_PI_2 && agent_rad > -M_PI)
    {
      irat_node->setRotation(
          irr::core::vector3df(0, 0, (irr::f32)(atan2(-facing_ratio * viewport_height, -viewport_width) * 180 / M_PI)));
    }
  }

  void update_viewport(int x, int y, int width, int height)
  {
    viewport_x = x;
    viewport_y = y;
    viewport_width = width;
    viewport_height = height;

    // setup an orthogonal matrix
    irr::scene::ICameraSceneNode * gt_map_camera = gt_map_scene->addCameraSceneNode(
        NULL, irr::core::vector3df((irr::f32)(0.5 * viewport_width), (irr::f32)(0.5 * viewport_height), (irr::f32)-40),
        irr::core::vector3df((irr::f32)(0.5 * viewport_width), (irr::f32)(0.5 * viewport_height), (irr::f32)0));
    irr::core::matrix4 proj;
    proj.buildProjectionMatrixOrthoLH((irr::f32)(1.0 * viewport_width), (irr::f32)(1.0 * viewport_height),
                                      (irr::f32)0.01, (irr::f32)100);
    gt_map_camera->setProjectionMatrix(proj, true);
  }

  void set_viewport()
  {
    gt_map_scene->getVideoDriver()->setViewPort(
        irr::core::rect<irr::s32>((irr::s32)viewport_x, (irr::s32)viewport_y, (irr::s32)(viewport_x + viewport_width),
                                  (irr::s32)(viewport_y + viewport_height)));
  }

  void update_scene()
  {
    gt_map_path->clearPoints();
    int numposes = map->get_num_poses();

    update_poses();
    update_current_pose(map->current_pose_id);

    

  }

  void draw_all()
  {
    device->run();
    driver->beginScene(true, true, irr::video::SColor(255, 0, 0, 0));
    gt_map_scene->drawAll();
    driver->endScene();
  }

  void screen_to_world(int x, int y, double & x_m, double & y_m)
  {
    x_m = (double)(x - viewport_x) / viewport_width * (max_x - min_x) + min_x;
    y_m = (double)max_y - (y - viewport_y) / viewport_height * (max_y - min_y);
  }

  void update_ptr(GroundTruth *in_map)
  {
    map = in_map;
  }

private:
  irr::scene::ISceneManager * gt_map_scene;
  PathNode * gt_map_path;
  irr::scene::IMeshSceneNode * irat_node;
  std::string media_path;
  std::string image_file;

  irr::IrrlichtDevice *device;
  irr::video::IVideoDriver * driver;
  irr::scene::ISceneManager * scene;

  double min_x;
  double max_x;
  double min_y;
  double max_y;

  double viewport_x, viewport_y, viewport_width, viewport_height;

  std::vector<irr::scene::IBillboardTextSceneNode*> numberNodes;

  ratslam::GroundTruth *map;
};

}
;
// namespace ratslam

#endif

