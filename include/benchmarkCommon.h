#pragma once

#include <array>
#include <fstream> 
#include <vector>
#include <chrono>
#include <ros/package.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_env.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection/collision_tools.h>


void publishMarkers(visualization_msgs::MarkerArray &markers, std::unique_ptr<ros::Publisher>& g_marker_array_publisher, visualization_msgs::MarkerArray& g_collision_points)
{
  // delete old markers
  if (!g_collision_points.markers.empty())
  {
    for (auto &marker : g_collision_points.markers)
      marker.action = visualization_msgs::Marker::DELETE;

    g_marker_array_publisher->publish(g_collision_points);
  }

  // move new markers into g_collision_points
  std::swap(g_collision_points.markers, markers.markers);

  // draw new markers (if there are any)
  if (!g_collision_points.markers.empty())
    g_marker_array_publisher->publish(g_collision_points);
}

void computeCollisionContactPoints(std::shared_ptr<planning_scene::PlanningScene> &planning_scene, moveit::core::RobotState &state, std::unique_ptr<ros::Publisher>& g_marker_array_publisher, visualization_msgs::MarkerArray& g_collision_points)
{
  collision_detection::CollisionRequest c_req;
  collision_detection::CollisionResult c_res;
  // c_req.group_name = robot.getGroupName();
  // ROS_INFO_STREAM("Group name: " << c_req.group_name);
  c_req.contacts = true;
  c_req.max_contacts = 100;
  c_req.max_contacts_per_pair = 5;
  c_req.verbose = false;

  // ROS_INFO_STREAM("Find " << g_planning_scene->getCollisionEnv()->getWorld()->getObjectIds().size() << " objects in the world");
  // auto world = g_planning_scene->getWorld()->getObjectIds();
  // for (auto it = world.begin(); it != world.end(); it++)
  // {
  //   ROS_INFO_STREAM("Object " << *it);
  //   auto shape = g_planning_scene->getWorld()->getObject(*it)->shapes_.front();
  //   ROS_INFO_STREAM("Shape " << shape->type);
  //   shape->print(std::cout);
  // }

  // auto robot_state = g_planning_scene->getCurrentStateNonConst();
  // for (int i = 0; i < 6; i++)
  // {
  //     ROS_INFO_STREAM("Joint " << i+1 << " is " << *robot_state.getJointPositions("jaka_joint_" + std::to_string(i + 1)));
  // }

  planning_scene->checkCollision(c_req, c_res, state);

  if (c_res.collision)
  {
    ROS_INFO_STREAM("COLLIDING contact_point_count=" << c_res.contact_count);
    if (c_res.contact_count > 0)
    {
      std_msgs::ColorRGBA color;
      color.r = 1.0;
      color.g = 0.0;
      color.b = 1.0;
      color.a = 0.5;
      visualization_msgs::MarkerArray markers;

      /* Get the contact points and display them as markers */
      collision_detection::getCollisionMarkersFromContacts(markers, "base", c_res.contacts, color,
                                                           ros::Duration(), // remain until deleted
                                                           0.01);           // radius
      publishMarkers(markers, g_marker_array_publisher, g_collision_points);
    }
  }
  else
  {
    ROS_INFO("Not colliding");

    // delete the old collision point markers
    visualization_msgs::MarkerArray empty_marker_array;
    publishMarkers(empty_marker_array, g_marker_array_publisher, g_collision_points);
  }
}


// template <int dof>
// static void poseToState(const std::array<double, dof> &pose, std::array<std::string, dof> jointNames, RobotState &state)
// {
//     for (int i = 0; i < dof; i++)
//     {
//         state.setJointPositions(jointNames[i], &pose[i]);
//     }
//     state.update();
// }

// template <int dof>
// void loadPoses(std::vector<RobotState> &states, std::vector<std::pair<RobotState, RobotState>> &statePairs, std::array<std::string, dof> jointNames, RobotState &templateState)
// {
//     std::vector<std::array<double, dof>> poses;
//     // Read poses from bin file as vector of arrays
//     std::string path = ros::package::getPath("rtcc_benchmark");
//     std::string dataDir = path + "/data/";

//     std::ifstream file(dataDir + "poses.bin", std::ios::binary);
//     if (file.is_open())
//     {
//         while (true)
//         {
//             std::array<float, dof> posef;
//             std::array<double, dof> pose;
//             file.read(reinterpret_cast<char *>(posef.data()), sizeof(float) * dof);
//             if (file.eof())
//             {
//                 break;
//             }

//             std::transform(posef.begin(), posef.end(), pose.begin(),
//                            [](float f)
//                            { return static_cast<double>(f); });
//             poses.push_back(pose);
//         }
//     }
//     file.close();
//     states.clear();
//     states.reserve(poses.size()); // Reserve space to avoid reallocations

//     // Define the lambda function outside std::transform
//     auto poseToRobotState = [&templateState, jointNames](const std::array<double, dof> &pose) -> RobotState
//     {
//         RobotState state(templateState);
//         poseToState(pose, jointNames, state);
//         return state;
//     };

//     std::transform(poses.begin(), poses.end(), std::back_inserter(states), poseToRobotState);

//     std::vector<std::array<double, dof>> trajPoses;
//     file.open(dataDir + "trajPoses.bin", std::ios::binary);
//     if (file.is_open())
//     {
//         while (true)
//         {
//             std::array<float, dof> posef;
//             std::array<double, dof> pose;
//             file.read(reinterpret_cast<char *>(pose.data()), sizeof(float) * dof);
//             if (file.eof())
//             {
//                 break;
//             }
//             std::transform(posef.begin(), posef.end(), pose.begin(),
//                            [](float f)
//                            { return static_cast<double>(f); });
//             trajPoses.push_back(pose);
//         }
//     }
//     file.close();

//     size_t trajSize = trajPoses.size() / 2;
//     statePairs.clear();
//     statePairs.reserve(trajSize); // Reserve space to avoid reallocations

//     for (int i = 0; i < trajSize; i++)
//     {
//         RobotState startState = poseToRobotState(trajPoses[i]);
//         RobotState endState = poseToRobotState(trajPoses[i + trajSize]);
//         statePairs.emplace_back(std::make_pair(std::move(startState), std::move(endState)));
//     }
// }