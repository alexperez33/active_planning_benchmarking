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

#include "ompl/base/Planner.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/geometric/planners/rrt/STRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <std_srvs/Trigger.h>

#include <ompl/config.h>
#include <iostream>
#include <fstream>

#include "tprm/obstacle_impl.h"
#include "tprm/temporal_prm.h"
#include <tprm/config.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct MovingCircle {
    MovingCircle(const Eigen::Vector3d& _center, double _radius, Eigen::Vector3d _velocity, int _id) : center(_center), radius(_radius), velocity(_velocity), id(_id) {}
    Eigen::Vector3d center;
    double radius;
    Eigen::Vector3d velocity;
    int id;
    
    Eigen::Vector3d getPosition(double time) const
    {
        Eigen::Vector3d current_position;

        current_position(0) = center(0) + velocity(0) * time;
        current_position(1) = center(1) + velocity(1) * time;
        current_position(2) = center(2) + velocity(2) * time;

        return current_position;
    }
};

class SpaceTimeMotionValidator : public ob::MotionValidator {

public:
    explicit SpaceTimeMotionValidator(const ob::SpaceInformationPtr &si) : MotionValidator(si),
      vMax_(si_->getStateSpace().get()->as<ob::SpaceTimeStateSpace>()->getVMax()),
      stateSpace_(si_->getStateSpace().get()) {};

    bool checkMotion(const ob::State *s1, const ob::State *s2) const override
    {
        // assume motion starts in a valid configuration, so s1 is valid
        if (!si_->isValid(s2)) {
            invalid_++;
            return false;
        }

        // check if motion is forward in time and is not exceeding the speed limit
        auto *space = stateSpace_->as<ob::SpaceTimeStateSpace>();
        auto deltaPos = space->distanceSpace(s1, s2);
        double time1 = s1->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        double time2 = s2->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        auto deltaT = time2 - time1;

        //std::cout << deltaPos << " " <<  deltaPos / deltaT << " " << vMax_ << std::endl;
        if (!(deltaT > 0 && deltaPos / deltaT <= vMax_)) {
            invalid_++;
            return false;
        }

        // check if the path between the states is unconstrained (perform interpolation)...
        int interpolation_amount = 10;
        std::vector<double> pos1;
        std::vector<double> pos2;
        for (int i = 0; i < 12; i++)
        {
            pos1.push_back( s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[i] );
            pos2.push_back( s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[i] );
        }

        for (int i = 0; i < interpolation_amount; i++)
        {
            
            //t = (time - t1) / (t2 - t1)
            //x = x1 + t * (x2 - x1)
            //y = y1 + t * (y2 - y1)
            //z = z1 + t * (z2 - z1)

            float percent = (float)i / interpolation_amount;

            auto space = si_->getStateSpace();
            ompl::base::ScopedState<> temp_state(space);
            std::vector<double> pos_interpol;
            double time_interpol;
            for (int i = 0; i < 12; i++)
            {
                pos_interpol.push_back( pos1[i] + percent * (pos2[i] - pos1[i]) );
                temp_state->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[i] = pos_interpol[i];
            }

            time_interpol = time1 + percent * deltaT;
            temp_state->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ob::TimeStateSpace::StateType>(1)->position = time_interpol;

            if (!si_->isValid(temp_state->as<ompl::base::SpaceTimeStateSpace::StateType>())) {
                invalid_++;
                return false;
            }
        }

        return true;
    }

    bool checkMotion(const ompl::base::State *, const ompl::base::State *,
                     std::pair<ob::State *, double> &) const override
    {
        std::cout << "throwing" << std::endl;
        throw ompl::Exception("SpaceTimeMotionValidator::checkMotion", "not implemented");
    }

private:
    double vMax_; // maximum velocity
    ob::StateSpace *stateSpace_; // the animation state space for distance calculation
};

class STValidityCheckerDynObst : public ompl::base::StateValidityChecker {
public:
    STValidityCheckerDynObst(const ompl::base::SpaceInformationPtr& si, std::vector<MovingCircle> _obstacles, double time,
    planning_scene::PlanningScene* _g_planning_scene, moveit::core::RobotModelConstPtr _kinematic_model, 
    moveit::core::RobotState* _kinematic_state)
        : ompl::base::StateValidityChecker(si), obstacles(_obstacles), m_time(time), g_planning_scene(_g_planning_scene),
        kinematic_model(_kinematic_model), kinematic_state(_kinematic_state) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ompl::base::State* state) const {
        bool valid = this->validity_check(state);
        return valid;
    }

    void set_time(double time) {
        m_time = time;
    }

    // Returns if there is a collision occuring with the given conditions
    double validity_check(const ompl::base::State* state) const {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ompl::base::RealVectorStateSpace::StateType* stateXD = 
            state->as<ob::SpaceTimeStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0);
        double time = state->as<ob::SpaceTimeStateSpace::StateType>()->as<ob::TimeStateSpace::StateType>(1)->position;

        // 1.  Extract the robot's (x,y) position from its state. Extract obstacle's positions at time m_time
        std::vector<double> pos;
        for (int i = 0; i < 12; i++)
        {
            pos.push_back(stateXD->values[i]);
        }

        // 2. Update g_planning_state with obstacles and the state of the robot
        update_planning_scene_robot(pos);
        for(int i = 0; i < obstacles.size(); i++)
        {
            update_planning_scene_obstacles(obstacles[i].getPosition(time), obstacles[i].radius, obstacles[i].id);
        }

        // 3. Run collision check function
        return checkForCollisions();
    }

    void update_planning_scene_robot(std::vector<double> joint_values) const
    {
        // TODO: make compatible with right arm group as well.
        // set joint group position
        std::vector<double> left_pose(joint_values.begin() + 0, joint_values.begin() + 6);
        std::vector<double> right_pose(joint_values.begin() + 6, joint_values.end());
        const moveit::core::JointModelGroup* joint_model_group_left = kinematic_model->getJointModelGroup("left_arm");
        const moveit::core::JointModelGroup* joint_model_group_right = kinematic_model->getJointModelGroup("right_arm");
        kinematic_state->setJointGroupPositions(joint_model_group_left, left_pose);
        kinematic_state->setJointGroupPositions(joint_model_group_right, right_pose);
    }

    void update_planning_scene_obstacles(Eigen::Vector3d position, float r, int id) const
    {
        // Add collision element
        moveit_msgs::CollisionObject collision_objects;
        collision_objects.operation = moveit_msgs::CollisionObject::ADD;
        collision_objects.header.stamp = ros::Time::now();
        collision_objects.header.frame_id = "world";
        collision_objects.id = std::to_string(id);

        geometry_msgs::Pose cylinderPose;
        cylinderPose.position.x = position(0);
        cylinderPose.position.y = position(1);
        cylinderPose.position.z = position(2);
        cylinderPose.orientation.x = 0;
        cylinderPose.orientation.y = 0;
        cylinderPose.orientation.z = 0;
        cylinderPose.orientation.w = 1;
        shape_msgs::SolidPrimitive cylinder_shape;
        cylinder_shape.type = shape_msgs::SolidPrimitive::SPHERE;
        cylinder_shape.dimensions.resize(3);
        cylinder_shape.dimensions[0] = r*2;
        cylinder_shape.dimensions[1] = r*2;
        cylinder_shape.dimensions[2] = r*2;
        collision_objects.primitives.push_back(cylinder_shape);
        collision_objects.primitive_poses.push_back(cylinderPose);

        g_planning_scene->processCollisionObjectMsg(collision_objects);
    }

    bool checkForCollisions() const
    {
        // Checking for Collisions
        collision_detection::CollisionRequest c_req;
        collision_detection::CollisionResult c_res;
        c_req.contacts = true;
        c_req.max_contacts = 100;
        c_req.max_contacts_per_pair = 5;
        c_req.verbose = false;
        g_planning_scene->checkCollision(c_req, c_res, *kinematic_state);
        //ROS_INFO_STREAM("COLLIDING contact_point_count: " << c_res.contact_count);

        return c_res.contact_count == 0;
    }

private:
    std::vector<MovingCircle> obstacles;
    double m_time;
    planning_scene::PlanningScene* g_planning_scene;
    moveit::core::RobotModelConstPtr kinematic_model;
    moveit::core::RobotState* kinematic_state;
};


class DualKinovaPlanner {

    private:

    // ros variables
    int counter;
    ros::Publisher pub;
    ros::Subscriber planning_scene_sub;
    ros::ServiceServer solve_service;
    ros::ServiceServer execute_service;
    ros::ServiceServer execute_obstacles_service;

    // space variables
    ros::Publisher vis_pub;
    std::vector<std::string> left_joint_list;
    std::vector<std::string> right_joint_list;
    std::vector<std::string> both_joint_list;
    robot_model_loader::RobotModelLoader robot_loader;
    planning_scene::PlanningScene* g_planning_scene;
    moveit::core::RobotModelConstPtr kinematic_model;
    moveit::core::RobotState* kinematic_state;

    // scenario variables
    std::vector<double> start_joint_pose_left;
    std::vector<double> goal_joint_pose_left;
    std::vector<double> start_joint_pose_right;
    std::vector<double> goal_joint_pose_right;
    std::string current_solver;
    std::vector<MovingCircle> obstacles;

    //result variables
    std::vector<std::vector<double>> spacetime_path;

    std::string results = "";
    double strrt_goal_travel_time = 0;
    double strrt_solution_found = false;

    tprm::TemporalPRM* _tprm = nullptr;

    public:
    DualKinovaPlanner(ros::NodeHandle *nh) {

        solve_service = nh->advertiseService("/plan_kinova", &DualKinovaPlanner::callback_plan_solve, this);
        execute_service = nh->advertiseService("/execute_kinova", &DualKinovaPlanner::execute_spacetime_path, this);
        execute_obstacles_service = nh->advertiseService("/execute_obstacles", &DualKinovaPlanner::execute_obstacles_only, this);
        pub = nh->advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 10);
        
        vis_pub = nh->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

        left_joint_list = { "left_joint_1",
                            "left_joint_2",
                            "left_joint_3",
                            "left_joint_4",
                            "left_joint_5",
                            "left_joint_6"
                          };
        right_joint_list = { "right_joint_1",
                             "right_joint_2",
                             "right_joint_3",
                             "right_joint_4",
                             "right_joint_5",
                             "right_joint_6"
                          };

        both_joint_list = { "left_joint_1",
                            "left_joint_2",
                            "left_joint_3",
                            "left_joint_4",
                            "left_joint_5",
                            "left_joint_6"
                            "right_joint_1",
                            "right_joint_2",
                            "right_joint_3",
                            "right_joint_4",
                            "right_joint_5",
                            "right_joint_6"
                          };

        robot_loader = robot_model_loader::RobotModelLoader("robot_description");
        kinematic_model = robot_loader.getModel();
        kinematic_state = new moveit::core::RobotState(kinematic_model);
        g_planning_scene = new planning_scene::PlanningScene(kinematic_model);
    }

    ~DualKinovaPlanner()
    {
        delete _tprm;
        delete kinematic_state;
        delete g_planning_scene;
    }

    bool callback_plan_solve(std_srvs::Trigger::Request &req, 
                              std_srvs::Trigger::Response &res)
    {
        if (current_solver == "tprm")
            solve_tprm(true, 8000, 3, "both_arms");
        else if (current_solver == "tprm_original")
            solve_tprm(false, 8000, 3, "both_arms");
        else if (current_solver == "strrt")
            solve_strrt(60, 15, "both_arms");
        else
            std::cout << "Solver unrecognized" << std::endl;
        return true;
    }

    bool execute_spacetime_path(std_srvs::Trigger::Request &req, 
                                std_srvs::Trigger::Response &res)
    {
        return execute_path();
    }

    bool execute_path()
    {
        // TODO: use two threads to run execute_left_spacetime_path and execute_right_spacetime_path
        // They should in theory keep the same pace as each other (consider using rate instead of sleep in while loop)
        // Only one of the functions should be in charge of obstacles
        
        double time_prev = 0;

        for (int i = 1; i < spacetime_path.size(); i++)
        {
            std::vector<double> spacetime_point_prev = spacetime_path[i-1];
            std::vector<double> spacetime_point = spacetime_path[i];

            ////////////////////

            // check if the path between the states is unconstrained (perform interpolation)...
            std::vector<double> pos1 = std::vector<double>(spacetime_point_prev.begin(), spacetime_point_prev.end()-1);
            std::vector<double> pos2 = std::vector<double>(spacetime_point.begin(), spacetime_point.end()-1);
            double t1 = spacetime_point_prev[spacetime_point_prev.size()-1];
            double t2 = spacetime_point[spacetime_point.size()-1];
            double t = t1;
            double delta = 0.01;

            while (t < t2)
            {
                
                //t = (time - t1) / (t2 - t1)
                //x = x1 + t * (x2 - x1)
                //y = y1 + t * (y2 - y1)
                //z = z1 + t * (z2 - z1)

                float percent = 1.0 - ((t2 - t) / (t2 - t1));

                std::vector<double> pos_interpol;
                double time_interpol;
                for (int i = 0; i < 12; i++)
                {
                    pos_interpol.push_back( pos1[i] + percent * (pos2[i] - pos1[i]) );
                }
                time_interpol = t1 + percent * (t2 - t1);

                // 2. Update g_planning_state with obstacles and the state of the robot
                std::vector<double> left_pose(pos_interpol.begin() + 0, pos_interpol.begin() + 6);
                std::vector<double> right_pose(pos_interpol.begin() + 6, pos_interpol.end());
                setCurrentJointState(left_pose, true, "left_arm");
                setCurrentJointState(right_pose, true, "right_arm");

                for(int j = 0; j < obstacles.size(); j++)
                {
                    add_obstacle(obstacles[j].getPosition(time_interpol), obstacles[j].radius, obstacles[j].id, true);
                }

                // 3. Run collision check function
                bool collision_occurred = !checkForCollisions();
                if (collision_occurred)
                {
                    return false;
                }

                t += delta;
                ros::Duration(delta).sleep();

            }

        }

        return true;
    }

    bool execute_obstacles_only(std_srvs::Trigger::Request &req, 
                                std_srvs::Trigger::Response &res)
    {

        double time_prev = 0;

        ////////////////////

        // check if the path between the states is unconstrained (perform interpolation)...
        double t1 = 0;
        double t2 = 20;
        double t = t1;
        double delta = 0.01;

        while (t < t2)
        {
            
            //t = (time - t1) / (t2 - t1)
            //x = x1 + t * (x2 - x1)
            //y = y1 + t * (y2 - y1)
            //z = z1 + t * (z2 - z1)

            float percent = 1.0 - ((t2 - t) / (t2 - t1));

            std::vector<double> pos_interpol;
            double time_interpol;
            time_interpol = t1 + percent * (t2 - t1);

            for(int j = 0; j < obstacles.size(); j++)
            {
                add_obstacle(obstacles[j].getPosition(time_interpol), obstacles[j].radius, obstacles[j].id, true);
            }

            // 3. Run collision check function
            checkForCollisions();

            t += delta;
            ros::Duration(delta).sleep();

        }

        return true;
    }

    const double* getCurrentJointState_Left()
    {
        const moveit::core::JointModel* model = kinematic_model->getJointModelGroup("left_arm")->getJointModel(left_joint_list[0]);
        const double* joint = kinematic_state->getJointPositions(model);
        return joint;
    }

    const double* getCurrentJointState_Right()
    {
        const moveit::core::JointModel* model = kinematic_model->getJointModelGroup("right_arm")->getJointModel(right_joint_list[0]);
        const double* joint = kinematic_state->getJointPositions(model);
        return joint;
    }

    void setCurrentJointState(std::vector<double> joint_values, bool set_on_rviz, std::string group = "left_arm")
    {

        // set joint group position
        const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group);
        kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

        if (set_on_rviz)
        {
            //std::cout << "setting joint state" << std::endl;
            // set visual position
            sensor_msgs::JointState joint_state_msg;
            
            if (group == "left_arm")
                joint_state_msg.name = left_joint_list;
            else if (group == "right_arm")
                joint_state_msg.name = right_joint_list;
            else if (group == "both_arms")
                joint_state_msg.name = both_joint_list;
            else
                return;

            joint_state_msg.position = joint_values;
            pub.publish(joint_state_msg);
        }

        //std::cout << "joint state set" << std::endl;
    }

    bool checkForCollisions()
    {
        // Checking for Collisions
        collision_detection::CollisionRequest c_req;
        collision_detection::CollisionResult c_res;
        c_req.contacts = true;
        c_req.max_contacts = 100;
        c_req.max_contacts_per_pair = 5;
        c_req.verbose = false;
        g_planning_scene->checkCollision(c_req, c_res, *kinematic_state);
        ROS_INFO_STREAM("COLLIDING contact_point_count: " << c_res.contact_count);

        return c_res.contact_count == 0;

    }

    void add_obstacle(Eigen::Vector3d position, float r, int id, bool set_on_rviz)
    {
        // Add collision element
        moveit_msgs::CollisionObject collision_objects;
        collision_objects.operation = moveit_msgs::CollisionObject::ADD;
        collision_objects.header.stamp = ros::Time::now();
        collision_objects.header.frame_id = "world";
        collision_objects.id = std::to_string(id);

        geometry_msgs::Pose cylinderPose;
        cylinderPose.position.x = position(0);
        cylinderPose.position.y = position(1);
        cylinderPose.position.z = position(2);
        cylinderPose.orientation.x = 0;
        cylinderPose.orientation.y = 0;
        cylinderPose.orientation.z = 0;
        cylinderPose.orientation.w = 1;
        shape_msgs::SolidPrimitive cylinder_shape;
        cylinder_shape.type = shape_msgs::SolidPrimitive::SPHERE;
        cylinder_shape.dimensions.resize(3);
        cylinder_shape.dimensions[0] = r*2;
        cylinder_shape.dimensions[1] = r*2;
        cylinder_shape.dimensions[2] = r*2;
        collision_objects.primitives.push_back(cylinder_shape);
        collision_objects.primitive_poses.push_back(cylinderPose);

        g_planning_scene->processCollisionObjectMsg(collision_objects);

        if (set_on_rviz)
        {
            // Add visual element
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time();

            marker.id = id;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = cylinderPose.position.x;
            marker.pose.position.y = cylinderPose.position.y;
            marker.pose.position.z = cylinderPose.position.z;
            marker.pose.orientation.x = cylinderPose.orientation.x;
            marker.pose.orientation.y = cylinderPose.orientation.y;
            marker.pose.orientation.z = cylinderPose.orientation.z;
            marker.pose.orientation.w = cylinderPose.orientation.w;
            marker.scale.x = r*2;
            marker.scale.y = r*2;
            marker.scale.z = r*2;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            vis_pub.publish( marker );
        }
    }

    void addFloor(float x, float y, int id, bool set_on_rviz)
    {
        // Add collision element
        moveit_msgs::CollisionObject collision_objects;
        collision_objects.operation = moveit_msgs::CollisionObject::ADD;
        collision_objects.header.stamp = ros::Time::now();
        collision_objects.header.frame_id = "world";
        collision_objects.id = std::to_string(id);

        geometry_msgs::Pose floorPose;
        floorPose.position.x = 0;
        floorPose.position.y = 0;
        floorPose.position.z = -0.1;
        floorPose.orientation.x = 0;
        floorPose.orientation.y = 0;
        floorPose.orientation.z = 0;
        floorPose.orientation.w = 1;
        shape_msgs::SolidPrimitive floorShape;
        floorShape.type = shape_msgs::SolidPrimitive::BOX;
        floorShape.dimensions.resize(3);
        floorShape.dimensions[0] = x;
        floorShape.dimensions[1] = y;
        floorShape.dimensions[2] = 0.1;
        collision_objects.primitives.push_back(floorShape);
        collision_objects.primitive_poses.push_back(floorPose);

        g_planning_scene->processCollisionObjectMsg(collision_objects);

        if (set_on_rviz)
        {
            // Add visual element
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time();

            marker.id = id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = floorPose.position.x;
            marker.pose.position.y = floorPose.position.y;
            marker.pose.position.z = floorPose.position.z;
            marker.pose.orientation.x = floorPose.orientation.x;
            marker.pose.orientation.y = floorPose.orientation.y;
            marker.pose.orientation.z = floorPose.orientation.z;
            marker.pose.orientation.w = floorPose.orientation.w;
            marker.scale.x = floorShape.dimensions[0];
            marker.scale.y = floorShape.dimensions[1];
            marker.scale.z = floorShape.dimensions[2];
            marker.color.a = 1;
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
            vis_pub.publish( marker );
        }
    }

    void define_scenario(std::string _solver,
                         std::vector<double>& _start_left,
                         std::vector<double>& _goal_left,
                         std::vector<double>& _start_right,
                         std::vector<double>& _goal_right,
                         std::vector<MovingCircle> _obstacles,
                         bool _use_current_position_as_start=false)
    {
        current_solver = _solver;
        obstacles = _obstacles;

        goal_joint_pose_left = _goal_left;
        goal_joint_pose_right = _goal_right;
        
        const double* current_state_left = getCurrentJointState_Left();
        const double* current_state_right = getCurrentJointState_Right();

        if (_use_current_position_as_start)
        {
            for (int i = 0; i < 6; i++)
            {
                start_joint_pose_left[i] = current_state_left[i];
                start_joint_pose_right[i] = current_state_right[i];
            }
        }
        else
        {
            start_joint_pose_left = _start_left;
            start_joint_pose_right = _start_right;
        }


        setCurrentJointState(start_joint_pose_left, true, "left_arm");
        setCurrentJointState(start_joint_pose_right, true, "right_arm");

        for(int j = 0; j < obstacles.size(); j++)
        {
            add_obstacle(obstacles[j].getPosition(0), obstacles[j].radius, obstacles[j].id, true);
        }

        // Optional floor
        addFloor(10, 10, -1, 1);

    }

    void change_solver(std::string _solver)
    {
        current_solver = _solver;
    }

    std::string solve_tprm(bool solve_updated_version, int numSamples, double costEdgeThreshold, std::string group = "left_arm", bool resolve = false)
    {
        tprm::HolonomicRobot::movement_speed = 0.25;  // rad/s
        spacetime_path.clear();

        uint dim = 6;
        if (group == "both_arms")
        {
            dim = 12;
            group = "arm";
        }
        
        tprm::TemporalPRM* tprm;
        if (resolve == false)
        {
            if (_tprm != nullptr)
                delete _tprm;
            
            _tprm = new tprm::TemporalPRM(g_planning_scene, kinematic_model, kinematic_state, dim, group, dim == 12);
        }
        else
        {
            _tprm->clearObstacles();
        }
        tprm = _tprm;

        std::cout << "make tprm" << std::endl;

        for (int i = 0; i < obstacles.size(); i++)
        {
            tprm->addDynamicObstacle(std::make_shared<tprm::DynamicSphereObstacle>(obstacles[i].center, obstacles[i].velocity, obstacles[i].radius, obstacles[i].id));
        }

        ros::Time begin;
        ros::Time end;
        double placing_samples_time = 0;
        double edge_build_time = 0;
        double solve_time = 0;

        if (resolve == false)
        {
            ///// PLACING SAMPLES //////
            begin = ros::Time::now();
            tprm->placeSamples(numSamples);
            end = ros::Time::now();
            
            placing_samples_time = (end - begin).sec + (end - begin).nsec * 1e-9;
            std::cout << "Placed " << numSamples << " samples in " << placing_samples_time << " seconds" << std::endl;
        

            ///// BUILDING PRM //////
            begin = ros::Time::now();
            tprm->buildPRM(costEdgeThreshold);
            end = ros::Time::now();

            edge_build_time = (end - begin).sec + (end - begin).nsec * 1e-9;
            std::cout << "Added " << tprm->getTemporalGraph().getNumEdges() << " edges in " << edge_build_time << " seconds" << std::endl;
        }
        else
        {
            ///// RECALCULATING TIME AVAILABILITY //////
            begin = ros::Time::now();
            tprm->recomputeTimeAvailabilities();
            end = ros::Time::now();
            
            placing_samples_time = (end - begin).sec + (end - begin).nsec * 1e-9;
            std::cout << "Recalculated Time Availability in " << placing_samples_time << " seconds" << std::endl;
        }

        ///// SOLVING PRM //////
        tprm::VectorNd start_tprm;
        tprm::VectorNd end_tprm;

        if (group == "left_arm")
        {
            start_tprm = tprm::VectorNd(start_joint_pose_left);
            end_tprm = tprm::VectorNd(goal_joint_pose_left);
        }
        else if (group == "right_arm")
        {
            start_tprm = tprm::VectorNd(start_joint_pose_right);
            end_tprm = tprm::VectorNd(goal_joint_pose_right);
        }
        else if (group == "arm")
        {
            std::vector<double> merged_start;
            merged_start.insert(merged_start.end(), start_joint_pose_left.begin(), start_joint_pose_left.end());
            merged_start.insert(merged_start.end(), start_joint_pose_right.begin(), start_joint_pose_right.end());
            
            std::vector<double> merged_end;
            merged_end.insert(merged_end.end(), goal_joint_pose_left.begin(), goal_joint_pose_left.end());
            merged_end.insert(merged_end.end(), goal_joint_pose_right.begin(), goal_joint_pose_right.end());

            start_tprm = tprm::VectorNd(merged_start);
            end_tprm = tprm::VectorNd(merged_end);
        }

        std::vector<tprm::PathResultEntry> path;

        begin = ros::Time::now();
        if (solve_updated_version)
        {
            int start_node_id = tprm->addSample(start_tprm, costEdgeThreshold);
            int end_node_id = tprm->addSample(end_tprm, costEdgeThreshold);
            path = tprm->getShortestPath(start_node_id, end_node_id, 0);
        }
        else
        {
            path = tprm->getShortestPath(start_tprm, end_tprm, 0);
        }
        end = ros::Time::now();

        solve_time = (end - begin).sec + (end - begin).nsec * 1e-9;
        std::cout << "Result provided in " << solve_time << " seconds" << std::endl;

        std::cout << "Total solve time is: " << placing_samples_time + edge_build_time + solve_time << " seconds" << std::endl;

        double travel_time = -1;
        if (path.empty()) {
            std::cout << "No path found" << std::endl;
        } else {
            /*for (auto& node : path) {
                std::cout << "Node: \n" << node.position.transpose() << " \nat time " << node.time << std::endl << std::endl;
            }*/
            travel_time = path.back().time;
        }

        for (auto& node : path)
        {
            std::vector<double> statetime_point;
            for (int i = 0; i < 12; i++)
            {
                statetime_point.push_back(node.position(i));
            }
            statetime_point.push_back(node.time);

            spacetime_path.push_back( statetime_point );
        }

        std::string res = std::to_string(numSamples) + ", " + 
                          std::to_string(tprm->getTemporalGraph().getNumEdges()) + ", " + 
                          std::to_string(placing_samples_time) + ", " + 
                          std::to_string(edge_build_time) + ", " + 
                          std::to_string(solve_time) + ", " +
                          std::to_string(placing_samples_time + edge_build_time + solve_time) + ", " +
                          std::to_string(travel_time);

        return res;
    }

    std::string solve_strrt(double max_solve_time, double goal_travel_time, std::string group = "left_arm")
    {
        spacetime_path.clear();
        strrt_solution_found = false;

        std::cout << "Entering strrt benchmark" << std::endl;
        // set maximum velocity
        double vMax = 0.5;
        double dim = 6;
        if (group == "both_arms")
        {
            dim = 12;
        }

        strrt_goal_travel_time = goal_travel_time;

        // construct the state space we are planning in
        auto vectorSpace(std::make_shared<ob::RealVectorStateSpace>(dim));
        auto space = std::make_shared<ob::SpaceTimeStateSpace>(vectorSpace, vMax);

        // set the bounds for R2 or R3
        ob::RealVectorBounds bounds(dim);
        bounds.setLow(-3.14);
        bounds.setHigh(3.14);
        vectorSpace->setBounds(bounds);

        // set time bounds. Planning with unbounded time is also possible when using ST-RRT*.
        // TODO: figure out how to make it unbounded
        space->setTimeBounds(0.0, 30.0);

        // create the space information class for the space
        ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);

        // set state validity checking for this space
        ompl::base::StateValidityCheckerPtr validityChecker = std::make_shared<STValidityCheckerDynObst>(si, obstacles, 0,
                                                                                                        g_planning_scene,
                                                                                                        kinematic_model,
                                                                                                        kinematic_state);
        si->setStateValidityChecker(validityChecker);
        si->setMotionValidator(std::make_shared<SpaceTimeMotionValidator>(si));

        // define a simple setup class
        ompl::geometric::SimpleSetup ss(si);

        ompl::base::ScopedState<> start(space);
        ompl::base::ScopedState<> goal(space);

        if (group == "left_arm")
        {
            for (int i = 0; i < start_joint_pose_left.size(); i++)
            {
                start->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[i] =
                    start_joint_pose_left[i];
            }

            for (int i = 0; i < goal_joint_pose_left.size(); i++)
            {
                goal->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[i] =
                    goal_joint_pose_left[i];
            }
        }
        if (group == "right_arm")
        {
            for (int i = 0; i < start_joint_pose_right.size(); i++)
            {
                start->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[i] =
                    start_joint_pose_right[i];
            }

            for (int i = 0; i < goal_joint_pose_right.size(); i++)
            {
                goal->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[i] =
                    goal_joint_pose_right[i];
            }
        }
        if (group == "both_arms")
        {
            for (int i = 0; i < start_joint_pose_left.size(); i++)
            {
                start->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[i] =
                    start_joint_pose_left[i];
            }

            for (int i = 0; i < goal_joint_pose_left.size(); i++)
            {
                std::cout << goal_joint_pose_left[i] << " ";
                goal->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[i] =
                    goal_joint_pose_left[i];
            }
            for (int i = 0; i < start_joint_pose_right.size(); i++)
            {
                start->as<ompl::base::SpaceTimeStateSpace::StateType>()->
                       as<ompl::base::RealVectorStateSpace::StateType>(0)->
                       values[i + start_joint_pose_left.size()] =
                       start_joint_pose_right[i];
            }

            for (int i = 0; i < goal_joint_pose_right.size(); i++)
            {
                std::cout << goal_joint_pose_right[i] << " ";
                goal->as<ompl::base::SpaceTimeStateSpace::StateType>()->
                      as<ompl::base::RealVectorStateSpace::StateType>(0)->
                      values[i + goal_joint_pose_left.size()] =
                      goal_joint_pose_right[i];
            }
        }


        // set the start and goal states
        ss.setStartAndGoalStates(start, goal);

        // construct the planner
        auto *strrtStar = new ompl::geometric::STRRTstar(si);

        // set planner parameters
        strrtStar->setRange(vMax);

        // set the used planner
        ss.setPlanner(ob::PlannerPtr(strrtStar));

        // terminating conditions
        ros::Time begin = ros::Time::now();
        std::function<bool()> terminating_function = [&]() { return strrt_terminating_condition(max_solve_time, begin); };
        ob::PlannerTerminationConditionFn fn(terminating_function);
        ob::PlannerTerminationCondition ptc(fn);

        std::function<void(const ob::Planner *, const std::vector<const ob::State *> &, const ob::Cost)> intermediate_function =
            [&](const ob::Planner * planner, const std::vector<const ob::State *> & states, const ob::Cost cost) 
                { return strrt_intermediate_condition(planner, states, cost); };
        ss.getProblemDefinition()->setIntermediateSolutionCallback(intermediate_function);

        // attempt to solve the problem within one second of planning time
        std::cout << "1" << std::endl;
        ob::PlannerStatus solved = ss.solve(ptc);
        std::cout << "2" << std::endl;
        ros::Time end = ros::Time::now();
        std::cout << solved << std::endl;
        
        double strrt_solve_time = (end - begin).sec + (end - begin).nsec * 1e-9;
        std::cout << "STRRT finished in " << strrt_solve_time << " seconds" << std::endl;

        if (solved == ob::PlannerStatus::EXACT_SOLUTION)
        {
            std::cout << "Found solution:" << std::endl;
            // print the path to screen
            ss.getSolutionPath().print(std::cout);
        }
        else
        {
            std::cout << "No solution found" << std::endl;
        }

        std::string res = std::to_string(max_solve_time) + ", " + 
                          std::to_string(strrt_goal_travel_time) + ", " +
                          std::to_string(strrt_solve_time) + ", ";
        
        if (solved == ob::PlannerStatus::EXACT_SOLUTION)
        {
            ompl::geometric::PathGeometric fullpath = ss.getSolutionPath();
            std::vector<ompl::base::State*> states = fullpath.getStates();
            std::vector<Eigen::Vector3d> fullpath_;
            for (ompl::base::State* state : states)
            {
                const ompl::base::SpaceTimeStateSpace::StateType* spaceTimeState =
                    state->as<ompl::base::SpaceTimeStateSpace::StateType>();

                const ompl::base::RealVectorStateSpace::StateType* spatialState =
                    spaceTimeState->as<ompl::base::RealVectorStateSpace::StateType>(0);
                
                std::vector<double> statetime_point;
                for (int i = 0; i < dim; i++)
                {
                    statetime_point.push_back(spatialState->values[i]);
                }
                statetime_point.push_back(spaceTimeState->as<ob::TimeStateSpace::StateType>(1)->position);

                spacetime_path.push_back( statetime_point );
            }

            if (!spacetime_path.empty() && 
                 spacetime_path[spacetime_path.size()-1].back() > spacetime_path[spacetime_path.size()-2].back())
            {
                res += std::to_string(spacetime_path.back().back());
            }
        }

        return res;
    }

    void strrt_intermediate_condition(const ob::Planner * planner, const std::vector<const ob::State *> & states, const ob::Cost cost)
    {

        // Check travel time condition
        std::cout << cost.value() << " " << strrt_goal_travel_time << std::endl;
        if (cost.value() < strrt_goal_travel_time)
        {   
            const ompl::base::State* last_state = states[states.size()-1];
            const ompl::base::State* second_to_last_state = states[states.size()-2];

            double spaceTimeState_last =
                last_state->as<ompl::base::SpaceTimeStateSpace::StateType>()->
                    as<ob::TimeStateSpace::StateType>(1)->position;
            double spaceTimeState_second_to_last =
                second_to_last_state->as<ompl::base::SpaceTimeStateSpace::StateType>()->
                    as<ob::TimeStateSpace::StateType>(1)->position;

            if (spaceTimeState_last > spaceTimeState_second_to_last && spaceTimeState_last < strrt_goal_travel_time)
            {
                strrt_solution_found = true;
            }
        }
    }

    bool strrt_terminating_condition(double solve_timeout, ros::Time begin)
    {

        // Check timeout condition
        ros::Time end = ros::Time::now();
        double strrt_solve_time = (end - begin).sec + (end - begin).nsec * 1e-9;
        if (strrt_solve_time > solve_timeout)
        {
            return true;
        }
        
        // Check travel time condition
        if (strrt_solution_found)
        {
            return true;

        }
        return false;
    }

    planning_scene::PlanningScene* getPlanningScene()
    {
        return g_planning_scene;
    }

    moveit::core::RobotModelConstPtr getKinematicModel()
    {
        return kinematic_model;
    }

    moveit::core::RobotState* getKinematicState()
    {
        return kinematic_state;
    }
};

int main (int argc, char **argv)
{

    // Initialize:
    ros::init(argc, argv, "dual_kinova_planner");
    ros::NodeHandle nh;
    DualKinovaPlanner nc = DualKinovaPlanner(&nh);
    ros::Duration(2).sleep();

    // Start Scenario 1:
    std::vector<double> start_left = {0.5, -0.5, 0.6, 0, 0, 0};
    std::vector<double> goal_left = {2, -0.5, 0.6, 0, 0, 0};
    std::vector<double> start_right = {-2, -0.5, 0.6, 0, 0, 0};
    std::vector<double> goal_right = {-0.5, -0.5, 0.6, 0, 0, 0};
    std::vector<MovingCircle> obstacles;

    for (int i = 0; i < 10; i++)
    {
        Eigen::Vector3d pos;
        pos << -0.5, 0.25, 0.1*i - 0.2;
        Eigen::Vector3d vel;
        vel << 0.1,0,0;
        obstacles.push_back(MovingCircle(pos, 0.04, vel, i));
    }
    nc.define_scenario("strrt", start_left, goal_left, start_right, goal_right, obstacles);
    ros::spin();

    //////// TESTING ///////////
    /*std::vector<int> sample_sizes = {200, 400, 600, 800, 1000, 2000, 4000, 6000};
    std::vector<double> edge_sizes = {2, 3, 4};
    std::vector<double> solve_times = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 20, 25, 30};
    std::vector<double> goal_travel_times = {5, 10, 15, 20, 25, 30, 35, 40};
    std::string results = "";
    std::string results_replan = "";
    std::string filePath = "";
    
    for (int trial = 0; trial < 10; trial++)
    {
    
        // TEST SETUP 1: New TPRM Algorithm
        results = "";
        results_replan = "";
        for (int i = 0; i < sample_sizes.size(); i++)
        {
            for (int j = 0; j < edge_sizes.size(); j++)
            {
                results += nc.solve_tprm(true, sample_sizes[i], edge_sizes[j], "both_arms") + ", " + std::to_string(nc.execute_path()) + "\n";
                std::cout << "--------------------------" << std::endl;
                results_replan += nc.solve_tprm(true, sample_sizes[i], edge_sizes[j], "both_arms", true) + ", " + std::to_string(nc.execute_path()) + "\n";
                std::cout << "--------------------------" << std::endl;
            }
        }

        filePath = "/home/asper/catkin_ws2/logs/data/dual_kinova/results_new_tprm_" + std::to_string(trial) + ".csv";
        std::ofstream ofs1(filePath.c_str(), std::ios_base::out );
        ofs1 << results;
        ofs1.close();

        filePath = "/home/asper/catkin_ws2/logs/data/dual_kinova/results_new_tprm_resolve_" + std::to_string(trial) + ".csv";
        std::ofstream ofs1_resolve(filePath.c_str(), std::ios_base::out );
        ofs1_resolve << results_replan;
        ofs1_resolve.close();

        // TEST SETUP 2: Original TPRM Algorithm
        results = "";
        results_replan = "";
        for (int i = 0; i < sample_sizes.size(); i++)
        {
            for (int j = 0; j < edge_sizes.size(); j++)
            {
                results += nc.solve_tprm(false, sample_sizes[i], edge_sizes[j], "both_arms") + ", " + std::to_string(nc.execute_path()) + "\n";
                std::cout << "--------------------------" << std::endl;
                results_replan += nc.solve_tprm(false, sample_sizes[i], edge_sizes[j], "both_arms", true) + ", " + std::to_string(nc.execute_path()) + "\n";
                std::cout << "--------------------------" << std::endl;
            }
        }

        filePath = "/home/asper/catkin_ws2/logs/data/dual_kinova/results_orig_tprm_" + std::to_string(trial) + ".csv";
        std::ofstream ofs2(filePath.c_str(), std::ios_base::out );
        ofs2 << results;
        ofs2.close();

        filePath = "/home/asper/catkin_ws2/logs/data/dual_kinova/results_orig_tprm_resolve_" + std::to_string(trial) + ".csv";
        std::ofstream ofs2_resolve(filePath.c_str(), std::ios_base::out );
        ofs2_resolve << results_replan;
        ofs2_resolve.close();

        // TEST SETUP 3: STRRT Algorithm: Constant max solve time, Variable target execution time
        nc.change_solver("strrt");

        results = "";
        for (int i = 0; i < goal_travel_times.size(); i++)
        {
            results += nc.solve_strrt(30, goal_travel_times[i], "both_arms") + ", " + std::to_string(nc.execute_path()) + "\n";
            std::cout << "--------------------------" << std::endl;
        }

        filePath = "/home/asper/catkin_ws2/logs/data/dual_kinova/results_strrt_target_exec_" + std::to_string(trial) + ".csv";
        std::ofstream ofs3(filePath.c_str(), std::ios_base::out );
        ofs3 << results;
        ofs3.close();
        
        // TEST SETUP 4: STRRT Algorithm: Variable defined solve time
        results = "";
        for (int i = 0; i < solve_times.size(); i++)
        {
            results += nc.solve_strrt(solve_times[i], 0, "both_arms") + ", " + std::to_string(nc.execute_path()) + "\n";
            std::cout << "--------------------------" << std::endl;
        }

        filePath = "/home/asper/catkin_ws2/logs/data/dual_kinova/results_strrt_target_solve_" + std::to_string(trial) + ".csv";
        std::ofstream ofs4(filePath.c_str(), std::ios_base::out );
        ofs4 << results;
        ofs4.close();
    }

    std::cout << "Testing Complete" << std::endl;
    std::system("killall -9 rosmaster");*/
}