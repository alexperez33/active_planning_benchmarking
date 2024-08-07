#include <kdl/chainiksolverpos_nr.hpp>

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <visualization_msgs/Marker.h>

#include <ompl/config.h>
#include <iostream>
#include <fstream>


class KDLTesting {

    private:

    public:
    KDLTesting(ros::NodeHandle *nh) {

    }
};

int main (int argc, char **argv)
{

    // Initialize:
    ros::init(argc, argv, "kdl_testing");
    ros::NodeHandle nh;
    KDLTesting nc = KDLTesting(&nh);
    ros::Duration(2).sleep();


    ros::spin();
}