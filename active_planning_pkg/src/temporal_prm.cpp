/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022,
 *  ETH Zurich - V4RL, Department of Mechanical and Process Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Matthias Busenhart
 *********************************************************************/

#include <omp.h>
#include <tprm/temporal_prm.h>

#include <random>

namespace tprm {

TemporalPRM::TemporalPRM(planning_scene::PlanningScene* _g_planning_scene, 
                         moveit::core::RobotModelConstPtr _kinematic_model, 
                         moveit::core::RobotState* _kinematic_state,
                         uint dimensions,
                         std::string _robot_group,
                         bool _dual_arm) :
                         m_graph(_g_planning_scene, _kinematic_model, _kinematic_state, dimensions, _robot_group, _dual_arm) {
    g_planning_scene = _g_planning_scene;
    kinematic_model = _kinematic_model;
    kinematic_state = _kinematic_state;
    dim = dimensions;
    robot_group = _robot_group;
    dual_arm = _dual_arm;

    m_environmentMin = VectorNd(-3.14, dim);
    m_environmentMax = VectorNd(3.14, dim);
    std::cout << "TPRM Object Built" << std::endl;
}

TemporalPRM::TemporalPRM(const VectorNd& environmentMin, const VectorNd& environmentMax) : 
                         m_environmentMin(environmentMin), m_environmentMax(environmentMax) {}

void TemporalPRM::setEnvironmentMin(const VectorNd& environmentMin) {
    m_environmentMin = environmentMin;
}
void TemporalPRM::setEnvironmentMax(const VectorNd& environmentMax) {
    m_environmentMax = environmentMax;
}

void TemporalPRM::addStaticObstacle(const std::shared_ptr<StaticObstacle>& obstacle) {
    m_staticObstacles.push_back(obstacle);
}

void TemporalPRM::addDynamicObstacle(const std::shared_ptr<DynamicObstacle>& obstacle) {
    m_dynamicObstacles.push_back(obstacle);
}

void TemporalPRM::clearObstacles()
{
    m_staticObstacles.clear();
    m_dynamicObstacles.clear();
}

void TemporalPRM::placeSamples(int numNodes) {
    std::cout << "placing samples" << std::endl;
    
    const VectorNd envSize = m_environmentMax - m_environmentMin;
    for (int i = 0; i < numNodes; i++) {
        VectorNd sample = VectorNd::Random(dim);
        m_graph.addNode(TemporalGraphNode(sample, getTimeAvailability(sample)));
    }
}

int TemporalPRM::addSample(VectorNd _sample, double costEdgeThreshold)
{
    int new_node_id = m_graph.addNode(TemporalGraphNode(_sample, getTimeAvailability(_sample)));

    const int loop_end = m_graph.getNumNodes();
#pragma omp parallel for schedule(dynamic, 1) shared(m_graph) firstprivate(costEdgeThreshold, m_staticObstacles, loop_end) default(none)
    for (int i = 0; i < loop_end; i++) {
        const TemporalGraphNode& node_i = m_graph.getNode(i);
        const TemporalGraphNode& node_j = m_graph.getNode(new_node_id);

        double cost = m_graph.getEdgeCost(TemporalGraphEdge(i, new_node_id));
        if (cost > costEdgeThreshold) {
            continue;
        }


#pragma omp critical
        m_graph.addEdge(TemporalGraphEdge(i, new_node_id));
    }

    return new_node_id;
}

void TemporalPRM::buildPRM(double costEdgeThreshold) {
    const int loop_end = m_graph.getNumNodes();
#pragma omp parallel for schedule(dynamic, 1) shared(m_graph) firstprivate(costEdgeThreshold, m_staticObstacles, loop_end) default(none)
    for (int i = 0; i < loop_end; i++) {
        for (int j = i + 1; j < loop_end; j++) {
            const TemporalGraphNode& node_i = m_graph.getNode(i);
            const TemporalGraphNode& node_j = m_graph.getNode(j);

            double cost = m_graph.getEdgeCost(TemporalGraphEdge(i, j));
            if (cost > costEdgeThreshold) {
                continue;
            }


#pragma omp critical
            m_graph.addEdge(TemporalGraphEdge(i, j));
        }
    }
}

void TemporalPRM::recomputeTimeAvailabilities() {
    for (int i = 0; i < m_graph.getNumNodes(); i++) {
        m_graph.getNode(i).time_availabilities = getTimeAvailability(m_graph.getNode(i).position);
    }
}

TemporalGraph& TemporalPRM::getTemporalGraph() {
    return m_graph;
}

std::vector<TimeAvailability> TemporalPRM::getTimeAvailability(const VectorNd& position) const {

    std::vector<double> collision_behavior = {0};
    std::vector<TimeAvailability> time_availabilities;
    double time_increment = 1;
    double time_limit = 30;
    double discretized_checks = time_limit / time_increment;
    double time = 0;
    bool previously_collided = false;

    for (int i = 0; i < discretized_checks; i++)
    {
        // Update obstacles and robot
        for (const auto& obstacle : m_dynamicObstacles) {
            update_planning_scene_obstacles(obstacle->getCOM(time), obstacle->m_radius, obstacle->m_id);
        }
        update_planning_scene_robot(position.convertToStdVect());

        bool isCollisionOccurring = checkForCollisions();

        // A collision has occurred
        if ( !previously_collided && isCollisionOccurring)
        {
            previously_collided = true;
            collision_behavior.push_back(time_increment * i);
        }

        // A collision stopped occurring
        else if ( previously_collided && !isCollisionOccurring)
        {
            previously_collided = false;
            collision_behavior.push_back(time_increment * i);
        }
        time += time_increment;
    }
    collision_behavior.push_back(std::numeric_limits<double>::infinity());

    return discreteAvailableToTimeInvervals(collision_behavior, time_increment, time_limit);

}

void TemporalPRM::update_planning_scene_robot(std::vector<double> joint_values) const
{
    // set joint group position
    if (not dual_arm)
    {
        const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(robot_group);
        kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
    } else
    {
        std::vector<double> left_pose(joint_values.begin() + 0, joint_values.begin() + dim/2);
        std::vector<double> right_pose(joint_values.begin() + dim/2, joint_values.end());
        const moveit::core::JointModelGroup* joint_model_group_left = kinematic_model->getJointModelGroup("left_" + robot_group);
        kinematic_state->setJointGroupPositions(joint_model_group_left, left_pose);
        const moveit::core::JointModelGroup* joint_model_group_right = kinematic_model->getJointModelGroup("right_" + robot_group);
        kinematic_state->setJointGroupPositions(joint_model_group_right, right_pose);
    }
}

void TemporalPRM::update_planning_scene_obstacles(Eigen::Vector3d position, float r, int id) const
{
    // Add collision element
    moveit_msgs::CollisionObject collision_objects;
    collision_objects.operation = moveit_msgs::CollisionObject::ADD;
    collision_objects.header.stamp = ros::Time::now();
    collision_objects.header.frame_id = "world";
    collision_objects.id = std::to_string(id);

    geometry_msgs::Pose spherePose;
    spherePose.position.x = position(0);
    spherePose.position.y = position(1);
    spherePose.position.z = position(2);
    spherePose.orientation.x = 0;
    spherePose.orientation.y = 0;
    spherePose.orientation.z = 0;
    spherePose.orientation.w = 1;
    shape_msgs::SolidPrimitive sphere_shape;
    sphere_shape.type = shape_msgs::SolidPrimitive::SPHERE;
    sphere_shape.dimensions.resize(3);
    sphere_shape.dimensions[0] = r*2;
    sphere_shape.dimensions[1] = r*2;
    sphere_shape.dimensions[2] = r*2;
    collision_objects.primitives.push_back(sphere_shape);
    collision_objects.primitive_poses.push_back(spherePose);

    g_planning_scene->processCollisionObjectMsg(collision_objects);
}

bool TemporalPRM::checkForCollisions() const
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

std::vector<TimeAvailability> TemporalPRM::discreteAvailableToTimeInvervals(std::vector<double> discreteAvailabilities, double time_increment, double time_limit) const
{
    std::vector<TimeAvailability> time_availabilities; 
    discreteAvailabilities.push_back(std::numeric_limits<double>::infinity());

    for (int i = 0; i < discreteAvailabilities.size(); i++)
    {
        if (i + 1 < discreteAvailabilities.size())
            time_availabilities.push_back({discreteAvailabilities[i], discreteAvailabilities[i+1] });
    }

    return time_availabilities;
}

std::vector<PathResultEntry> TemporalPRM::getShortestPath(const VectorNd& start, const VectorNd& goal, double timeStart) const {
    
    int closest_start_id = m_graph.getClosestNode(start, timeStart, m_dynamicObstacles, false);
    int closest_goal_id = m_graph.getClosestNode(goal, timeStart, m_dynamicObstacles, false);

    if (closest_start_id == -1 || closest_goal_id == -1)
    {
        return {};
    }

    const TemporalGraphNode& start_node = m_graph.getNode(closest_start_id);
    const TemporalGraphNode& goal_node = m_graph.getNode(closest_goal_id);

    // try to connect start and goal to their closest nodes

    // check dynamic obstacles
    TemporalGraphNode tmp_start(start, getTimeAvailability(start));
    if (!tmp_start.isActiveAt(timeStart)) {
        return {};
    }

    double time_to_closest_start = (start - start_node.position).norm() / HolonomicRobot::movement_speed;

    auto path = m_graph.getShortestPath(closest_start_id, closest_goal_id, timeStart + time_to_closest_start, m_dynamicObstacles, false);
    if (path.empty()) {
        return {};
    }

    TemporalGraphNode tmp_goal(goal, getTimeAvailability(goal));
    double time_from_closest_goal = (goal - goal_node.position).norm() / HolonomicRobot::movement_speed;
    if (!tmp_goal.isActiveAt(path.back().time + time_from_closest_goal)) {
        return {};
    }

    std::vector<PathResultEntry> result;
    // push back custom start
    result.push_back(PathResultEntry(start, timeStart));
    for (const auto& entry : path) {
        result.push_back(PathResultEntry(m_graph.getNode(entry.node_id).position, entry.time));
    }
    result.push_back(PathResultEntry(goal, path.back().time + time_from_closest_goal));

    return result;
}

std::vector<PathResultEntry> TemporalPRM::getShortestPath(const int start, const int goal, double timeStart) const {
    
    auto path = m_graph.getShortestPath(start, goal, timeStart, m_dynamicObstacles, true);
    if (path.empty()) {
        return {};
    }

    std::vector<PathResultEntry> result;
    // push back custom start
    for (const auto& entry : path) {
        result.push_back(PathResultEntry(m_graph.getNode(entry.node_id).position, entry.time));
    }

    return result;
}

} /* namespace tprm */