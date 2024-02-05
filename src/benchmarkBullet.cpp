#include <vector>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include "interactive_robot.h"

#include <array>

auto g_planning_scene = std::unique_ptr<planning_scene::PlanningScene>();

// Temp
auto g_marker_array_publisher = std::unique_ptr<ros::Publisher>();
auto g_col_marker_publisher = std::unique_ptr<ros::Publisher>();
shapes::ShapePtr g_world_cube_shape;
visualization_msgs::MarkerArray g_collision_points;
visualization_msgs::MarkerArray g_collision_objs;
static int colObjCount = 0;
const double BOX_SIZE = 0.1;

// void publishMarkers(visualization_msgs::MarkerArray &markers)
// {
//   // delete old markers
//   if (!g_collision_points.markers.empty())
//   {
//     for (auto &marker : g_collision_points.markers)
//       marker.action = visualization_msgs::Marker::DELETE;

//     g_marker_array_publisher->publish(g_collision_points);
//   }

//   // move new markers into g_collision_points
//   std::swap(g_collision_points.markers, markers.markers);

//   // draw new markers (if there are any)
//   if (!g_collision_points.markers.empty())
//     g_marker_array_publisher->publish(g_collision_points);
// }

// void computeCollisionContactPoints(InteractiveRobot &robot)
// {
//   collision_detection::CollisionRequest c_req;
//   collision_detection::CollisionResult c_res;
//   c_req.group_name = robot.getGroupName();
//   ROS_INFO_STREAM("Group name: " << c_req.group_name);
//   c_req.contacts = true;
//   c_req.max_contacts = 100;
//   c_req.max_contacts_per_pair = 5;
//   c_req.verbose = false;

//   ROS_INFO_STREAM("Find " << g_planning_scene->getCollisionEnv()->getWorld()->getObjectIds().size() << " objects in the world");
//   auto world = g_planning_scene->getWorld()->getObjectIds();
//   for (auto it = world.begin(); it != world.end(); it++)
//   {
//     ROS_INFO_STREAM("Object " << *it);
//     auto shape = g_planning_scene->getWorld()->getObject(*it)->shapes_.front();
//     ROS_INFO_STREAM("Shape " << shape->type);
//     shape->print(std::cout);
//   }

//   // auto robot_state = g_planning_scene->getCurrentStateNonConst();
//   // for (int i = 0; i < 6; i++)
//   // {
//   //     ROS_INFO_STREAM("Joint " << i+1 << " is " << *robot_state.getJointPositions("jaka_joint_" + std::to_string(i + 1)));
//   // }

//   g_planning_scene->checkCollision(c_req, c_res, *robot.robotState());

//   if (c_res.collision)
//   {
//     ROS_INFO_STREAM("COLLIDING contact_point_count=" << c_res.contact_count);
//     if (c_res.contact_count > 0)
//     {
//       std_msgs::ColorRGBA color;
//       color.r = 1.0;
//       color.g = 0.0;
//       color.b = 1.0;
//       color.a = 0.5;
//       visualization_msgs::MarkerArray markers;

//       /* Get the contact points and display them as markers */
//       collision_detection::getCollisionMarkersFromContacts(markers, "base", c_res.contacts, color,
//                                                            ros::Duration(), // remain until deleted
//                                                            0.01);           // radius
//       publishMarkers(markers);
//     }
//   }
//   else
//   {
//     ROS_INFO("Not colliding");

//     // delete the old collision point markers
//     visualization_msgs::MarkerArray empty_marker_array;
//     publishMarkers(empty_marker_array);
//   }

//   g_marker_array_publisher->publish(g_collision_objs);
// }
// TODO: Remove Temp

void add_cube(std::shared_ptr<planning_scene::PlanningScene> &planning_scene, std::array<double, 6> &cubeInfo, std::string name)
{

  // Create a shared pointer to a new box shape
  shapes::ShapePtr cube = std::make_shared<shapes::Box>(cubeInfo[0], cubeInfo[1], cubeInfo[2]);
  // Define the pose of the cube
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(cubeInfo[3], cubeInfo[4], cubeInfo[5]);

  // Add the cube to the planning scene
  planning_scene->getWorldNonConst()->addToObject(name, cube, pose);

  // Add to marker array
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base";
  marker.header.stamp = ros::Time::now();
  marker.ns = "collision_objects" + std::to_string(++colObjCount);
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = cubeInfo[3];
  marker.pose.position.y = cubeInfo[4];
  marker.pose.position.z = cubeInfo[5];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = cubeInfo[0];
  marker.scale.y = cubeInfo[1];
  marker.scale.z = cubeInfo[2];
  marker.color.a = 0.8; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration();
  g_collision_objs.markers.push_back(marker);
}

void setCollisionScene(std::shared_ptr<planning_scene::PlanningScene> &planning_scene)
{
  std::array<double, 6> cubeInfo = {0.6, 1.2, 0.74, 0, -0.45, -0.45};
  add_cube(planning_scene, cubeInfo, "table1");
  std::array<double, 6> cubeInfo2 = {0.51, 0.52, 1, 0.46, 0.76, -0.24};
  add_cube(planning_scene, cubeInfo2, "table2");
  std::array<double, 6> cubeInfo3 = {0.5, 0.7, 0.8, -1.77, -0.1, -0.34};
  add_cube(planning_scene, cubeInfo3, "table3");
  std::array<double, 6> cubeInfo4 = {0.1, 1.2, 0.8, -0.25, -0.45, -0.4};
  add_cube(planning_scene, cubeInfo4, "table4");
  std::array<double, 6> cubeInfo5 = {0.4, 1.0, 0.8, 0, -0.6, -0.4};
  add_cube(planning_scene, cubeInfo5, "table5");
  std::array<double, 6> cubeInfo6 = {0.4, 0.75, 0.85, 0, -0.825, -0.315};
  add_cube(planning_scene, cubeInfo6, "table6");
  std::array<double, 6> cubeInfo7 = {0.1, 1.2, 0.8, 0.25, -0.45, -0.4};
  add_cube(planning_scene, cubeInfo7, "table7");

  // std::array<double, 6> cubeInfo8 = {3,0.1,2.5, -0.75, 1, 0.5};
  // add_cube(cubeInfo8, "wall1");
  // std::array<double, 6> cubeInfo9 = {0.1,2.6,2.5, 0.84, -0.3, 0.5};
  // add_cube(cubeInfo9, "wall2");
  // std::array<double, 6> cubeInfo10 = {3,2.9,0.1, -0.75,-0.45,-0.79};
  // add_cube(cubeInfo10, "floor");
  // std::array<double, 6> cubeInfo11 = {3,2.9,0.1, -0.75,-0.45,1.5};
  // add_cube(cubeInfo11, "ceiling");
  // std::array<double, 6> cubeInfo12 = {0.9,0.4,2.5, 0.39,-1.8,0.5};
  // add_cube(cubeInfo12, "wall3");
  // std::array<double, 6> cubeInfo13 = {3,0.1,2.5, -0.75,-1.9,0.5};
  // add_cube(cubeInfo13, "wall4");

  g_col_marker_publisher->publish(g_collision_objs);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "benchmarkBullet");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // robot_model_loader::RobotModelLoader rm_loader_("robot_description");
  // moveit::core::RobotModelPtr robot_model_ = rm_loader_.getModel();
  // if (!robot_model_)
  // {
  //     ROS_ERROR("Could not load robot description");
  // }

  // g_planning_scene = std::make_unique<planning_scene::PlanningScene>(robot_model_);
  // g_planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(),
  //                                              true /* exclusive */);

  robot_model_loader::RobotModelLoader rm_loader_("robot_description");

  robot_model::RobotModelPtr robot_model = rm_loader_.getModel();
  ros::Publisher robot_state_publisher_(node_handle.advertise<moveit_msgs::DisplayRobotState>("benchmarkBullet/interactive_robot_state", 1));
  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(),
                                             /* exclusive = */ true);
  // Temp
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  ros::Publisher marker_publisher(node_handle.advertise<visualization_msgs::Marker>("interactive_robot_markers", 1));
  visual_tools.setRobotStateTopic("interactive_robot_state");

  g_col_marker_publisher = std::make_unique<ros::Publisher>(
      node_handle.advertise<visualization_msgs::MarkerArray>("col_scene_marray", 100));

  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to start the continuous collision detection demo.");
  ROS_INFO("Shutting down the interactive interactive_robot...");

  setCollisionScene(planning_scene);

  moveit::core::RobotState state = planning_scene->getCurrentStateNonConst();
  state.setToDefaultValues();

  double joint1 = 1.7;
  double joint2 = 1.024;
  double joint3 = 1.8224;
  double joint4 = 1.8918;
  double joint5 = -1.57;
  double joint6 = 0.7116;
  state.setJointPositions("jaka_joint_1", &joint1);
  state.setJointPositions("jaka_joint_2", &joint2);
  state.setJointPositions("jaka_joint_3", &joint3);
  state.setJointPositions("jaka_joint_4", &joint4);
  state.setJointPositions("jaka_joint_5", &joint5);
  state.setJointPositions("jaka_joint_6", &joint6);
  state.update();

  moveit_msgs::DisplayRobotState msg;
  robot_state::robotStateToRobotStateMsg(state, msg.state);
  visual_tools.publishRobotState(state);

  // {
  //   // BEGIN_TUTORIAL
  //   // The code starts with creating an interactive robot and a new planning scene.
  //   InteractiveRobot interactive_robot("robot_description", "benchmarkBullet/interactive_robot_state");
  //   g_planning_scene = std::make_unique<planning_scene::PlanningScene>(interactive_robot.robotModel());

  //   // Changing the collision detector to Bullet
  //   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //   // The active collision detector is set from the planning scene using the specific collision detector allocator for
  //   // Bullet. The second argument indicates that Bullet will be the exclusive collision detection algorithm; the
  //   // default FCL will not be available anymore. Having one exclusive collision detection algorithm helps performance
  //   // a bit and is much more common.
  //   g_planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(),
  //                                                true /* exclusive */);
  //   // For understanding the interactive interactive_robot, please refer to the Visualizing Collisions tutorial.
  //   // CALL_SUB_TUTORIAL CCD
  //   // CALL_SUB_TUTORIAL CCD_2
  //   // END_TUTORIAL

  //   // Eigen::Isometry3d world_cube_pose;
  //   // double world_cube_size;
  //   // interactive_robot.getWorldGeometry(world_cube_pose, world_cube_size);
  //   // g_world_cube_shape = std::make_shared<shapes::Box>(world_cube_size, world_cube_size, world_cube_size);
  //   // g_planning_scene->getWorldNonConst()->addToObject("world_cube", g_world_cube_shape, world_cube_pose);

  //   // Create a marker array publisher for publishing contact points
  //   g_marker_array_publisher = std::make_unique<ros::Publisher>(
  //       node_handle.advertise<visualization_msgs::MarkerArray>("interactive_robot_marray", 100));

  //   interactive_robot.setUserCallback(computeCollisionContactPoints);

  //   visual_tools.loadRemoteControl();
  //   visual_tools.prompt(
  //       "Press 'next' in the RvizVisualToolsGui window to start the continuous collision detection demo.");
  //   ROS_INFO("Shutting down the interactive interactive_robot...");

  //   // remove all collision markers
  //   if (!g_collision_points.markers.empty())
  //   {
  //     for (auto &marker : g_collision_points.markers)
  //       marker.action = visualization_msgs::Marker::DELETE;

  //     g_marker_array_publisher->publish(g_collision_points);
  //   }

  //   visual_tools.deleteAllMarkers();
  // }
  // TODO: Remove Temp
}