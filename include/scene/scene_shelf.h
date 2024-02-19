#pragma once

#include "scene_utils.h"
#include <ros/package.h>
#include <geometric_shapes/mesh_operations.h>
#include <moveit/planning_scene/planning_scene.h>

void setCollisionScene(std::shared_ptr<planning_scene::PlanningScene> planning_scene)
{
    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/bin/bin.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();

        // Set the translation part
        meshPose.translation() << 0, -0.6, 0;

        add_mesh(planning_scene, meshPath, "bin1", meshPose);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/bin/bin.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();

        // Set the translation part
        meshPose.translation() << 0, 0.6, 0;

        add_mesh(planning_scene, meshPath, "bin2", meshPose);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/shelves/shelves.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();

        // Set the translation part
        meshPose.translation() << 0.85, 0, 0.4;

        add_mesh(planning_scene, meshPath, "shelf", meshPose);
    }
}

//   Bob:
//     pose: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
//     file_path: "scene/Bob.obj"
//   Face1:
//     pose: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
//     file_path: "scene/Face1.obj"
//   Face2:
//     pose: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
//     file_path: "scene/Face2.obj"
//   Face3:
//     pose: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
//     file_path: "scene/Face3.obj"
//   Cow:
//     pose: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
//     file_path: "scene/Cow.obj"
//   City:
//     pose: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
//     file_path: "scene/City.obj"
//   Bridge:
//     pose: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
//     file_path: "scene/Bridge.obj"
//   Fish:
//     pose: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
//     file_path: "scene/Fish.obj"
//   Sheep:
//     pose: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
//     file_path: "scene/Sheep.obj"
//   Rose:
//     pose: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
//     file_path: "scene/Rose.obj"
//   Board:
//     pose: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
//     file_path: "scene/Snakeboard.obj"
//   Yeah:
//     pose: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
//     file_path: "scene/Yeah.obj"
void setCollisionScene(std::shared_ptr<planning_scene::PlanningScene> planning_scene, visualization_msgs::MarkerArray &g_collision_objs, std::unique_ptr<ros::Publisher>& g_col_marker_publisher)
{
    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/bin/bin.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();

        // Set the translation part
        // meshPose.translation() << 0, -0.6, 0;
        meshPose.translation() << 0, -0.6, 0;

        add_mesh(planning_scene, meshPath, "bin1", meshPose, g_collision_objs);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/bin/bin.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();

        // Set the translation part
        // meshPose.translation() << 0, 0.6, 0;
        meshPose.translation() << 0, 0.6, 0;

        add_mesh(planning_scene, meshPath, "bin2", meshPose, g_collision_objs);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/shelves/shelves.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();

        // Set the translation part
        // meshPose.translation() << 0.85, 0, 0.4;
        meshPose.translation() << 0.45, 0, 0.4;

        add_mesh(planning_scene, meshPath, "shelf", meshPose, g_collision_objs);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/Bob.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();
        meshPose.translation() << -0.4, 0, 0;

        add_mesh(planning_scene, meshPath, "Bob", meshPose, g_collision_objs);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/Face1.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();
        meshPose.translation() << -0.4, 0, 0;

        add_mesh(planning_scene, meshPath, "Face1", meshPose, g_collision_objs);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/Face2.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();
        meshPose.translation() << -0.4, 0, 0;

        add_mesh(planning_scene, meshPath, "Face2", meshPose, g_collision_objs);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/Face3.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();
        meshPose.translation() << -0.4, 0, 0;

        add_mesh(planning_scene, meshPath, "Face3", meshPose, g_collision_objs);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/Cow.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();
        meshPose.translation() << -0.4, 0, 0;

        add_mesh(planning_scene, meshPath, "Cow", meshPose, g_collision_objs);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/City.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();
        meshPose.translation() << -0.4, 0, 0;

        add_mesh(planning_scene, meshPath, "City", meshPose, g_collision_objs);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/Bridge.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();
        meshPose.translation() << -0.4, 0, 0;

        add_mesh(planning_scene, meshPath, "Bridge", meshPose, g_collision_objs);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/Fish.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();
        meshPose.translation() << -0.4, 0, 0;

        add_mesh(planning_scene, meshPath, "Fish", meshPose, g_collision_objs);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/Sheep.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();
        meshPose.translation() << -0.4, 0, 0;

        add_mesh(planning_scene, meshPath, "Sheep", meshPose, g_collision_objs);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/Rose.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();
        meshPose.translation() << -0.4, 0, 0;

        add_mesh(planning_scene, meshPath, "Rose", meshPose, g_collision_objs);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/Snakeboard.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();
        meshPose.translation() << -0.4, 0, 0;

        add_mesh(planning_scene, meshPath, "Board", meshPose, g_collision_objs);
    }

    {
        std::string meshPath = "file://" + ros::package::getPath("rtcc_benchmark") + "/data/assets/Yeah.obj";
        Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();
        meshPose.translation() << -0.4, 0, 0;

        add_mesh(planning_scene, meshPath, "Yeah", meshPose, g_collision_objs);
    }

    g_col_marker_publisher->publish(g_collision_objs);
}