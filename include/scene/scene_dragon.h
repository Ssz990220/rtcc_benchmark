#pragma once

#include "scene_utils.h"
#include <ros/package.h>
#include <geometric_shapes/mesh_operations.h>
#include <moveit/planning_scene/planning_scene.h>

void setCollisionScene(std::shared_ptr<planning_scene::PlanningScene> planning_scene, visualization_msgs::MarkerArray &g_collision_objs, std::unique_ptr<ros::Publisher> &g_col_marker_publisher)
{
    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/dragon.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();

        add_mesh(planning_scene, meshPath, "dragon", meshPose, g_collision_objs);
    }

    g_col_marker_publisher->publish(g_collision_objs);
}