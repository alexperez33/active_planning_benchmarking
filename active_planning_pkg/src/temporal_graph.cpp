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

#include <tprm/temporal_graph.h>

#include <deque>
#include <unordered_map>
#include <unordered_set>

namespace tprm {

TimeAvailability::TimeAvailability(double start_time, double end_time) : start(start_time), end(end_time) {
    if (start > end) {
        throw std::invalid_argument("start_time > end_time");
    }
    if (start < 0.0) {
        throw std::invalid_argument("start_time < 0.0");
    }
    if (end < 0.0) {
        throw std::invalid_argument("end_time < 0.0");
    }
}

bool TemporalGraphNode::isActiveAt(double time) const {
    if (time < 0.0) {
        throw std::invalid_argument("time < 0.0");
    }
    for (const TimeAvailability& time_availability : time_availabilities) {
        if (time >= time_availability.start && time <= time_availability.end) {
            return true;
        }
    }
    return false;
}

int TemporalGraphEdge::getOtherNodeId(int node_id) const {
    if (node_id == node_a_id) {
        return node_b_id;
    } else if (node_id == node_b_id) {
        return node_a_id;
    } else {
        throw std::runtime_error("Node id not found in edge");
    }
}

TemporalGraph::TemporalGraph() {
    m_edge_cost_function = std::bind(&TemporalGraph::defaultEdgeCostFunction, this, std::placeholders::_1);
}

TemporalGraph::TemporalGraph(planning_scene::PlanningScene* _g_planning_scene, 
                            moveit::core::RobotModelConstPtr _kinematic_model, 
                            moveit::core::RobotState* _kinematic_state,
                            uint dimensions,
                            std::string _robot_group,
                            bool _dual_arm) {
    m_edge_cost_function = std::bind(&TemporalGraph::defaultEdgeCostFunction, this, std::placeholders::_1);
    g_planning_scene = _g_planning_scene;
    kinematic_model = _kinematic_model;
    kinematic_state = _kinematic_state;
    dim = dimensions;
    robot_group = _robot_group;
    dual_arm = _dual_arm;
}

void TemporalGraph::clear() {
    m_nodes.clear();
    m_edges.clear();
}

int TemporalGraph::addNode(const TemporalGraphNode& node) {
    m_nodes.push_back(node);
    return m_nodes.size() - 1;
}

int TemporalGraph::addEdge(const TemporalGraphEdge& edge) {
    m_edges.push_back(edge);
    return m_edges.size() - 1;
}

const TemporalGraphNode& TemporalGraph::getNode(int node_id) const {
    if (node_id < 0 || node_id >= m_nodes.size()) {
        throw std::runtime_error("Node id not found");
    }
    return m_nodes[node_id];
}

TemporalGraphNode& TemporalGraph::getNode(int node_id) {
    if (node_id < 0 || node_id >= m_nodes.size()) {
        throw std::runtime_error("Node id not found");
    }
    return m_nodes[node_id];
}

const TemporalGraphEdge& TemporalGraph::getEdge(int edge_id) const {
    if (edge_id < 0 || edge_id >= m_edges.size()) {
        throw std::runtime_error("Edge id not found");
    }
    return m_edges[edge_id];
}

int TemporalGraph::getNumNodes() const {
    return m_nodes.size();
}

int TemporalGraph::getNumEdges() const {
    return m_edges.size();
}

int TemporalGraph::getClosestNode(const VectorNd& position, double current_time, 
                                  const std::vector<std::shared_ptr<tprm::DynamicObstacle>>& obstacles,
                                  bool account_for_obstacles) const {
    int closest_node_id = -1;
    double closest_node_distance = std::numeric_limits<double>::max();
    for (int node_id = 0; node_id < m_nodes.size(); node_id++) {
        double distance = (m_nodes[node_id].position - position).norm();
        double edge_time = distance / HolonomicRobot::movement_speed;
        
        if (account_for_obstacles)
        {
            if ( ! interpolation_collision_check(position, m_nodes[node_id].position, current_time, edge_time, obstacles) )
                continue;
        }
        
        
        if (distance < closest_node_distance) {
            closest_node_distance = distance;
            closest_node_id = node_id;
        }
    }
    return closest_node_id;
}

/**
 * @brief Helper struct for special double.
 * 
 * This struct allows to initialize an array / list / queue with infinite values.
 * Used in TemporalGraph::getShortestPath
 */
struct CustomDouble {
    double value = std::numeric_limits<double>::infinity();
};

std::vector<GraphPathResultEntry> TemporalGraph::getShortestPath(int start_node_id, int end_node_id, double start_time, 
                                                                 const std::vector<std::shared_ptr<tprm::DynamicObstacle>>& obstacles,
                                                                 bool interpolate_edges) const {

    std::vector<std::vector<TemporalGraphEdge>> edge_buckets(
        m_nodes.size(), std::vector<TemporalGraphEdge>());  // bucket for each node. Each bucket contains all edges that start at the node
    for (const TemporalGraphEdge& edge : m_edges) {
        edge_buckets[edge.node_a_id].push_back(edge);
        edge_buckets[edge.node_b_id].push_back(edge);
    }

    std::unordered_map<int, int> predecessor_map;   // predecessor of each node
    std::unordered_map<int, CustomDouble> g_costs;  // g-cost of each node
    std::unordered_map<int, CustomDouble> f_costs;  // f-cost of each node

    const TemporalGraphNode& goal_node = m_nodes[end_node_id];

    std::function<double(int)> heuristic_function = [&](int node_id) { return m_edge_cost_function(TemporalGraphEdge(node_id, end_node_id)); };

    // sort function for the open queue
    auto compare = [&](int a, int b) { return f_costs[a].value < f_costs[b].value; };

    std::deque<int> open_queue;
    std::unordered_set<int> open_queue_set;
    std::unordered_set<int> closed_list_set;

    std::unordered_map<int, CustomDouble> arrival_time;

    g_costs[start_node_id].value = 0.;  // init this node by access (--> inf value) and set 0
    f_costs[start_node_id].value = heuristic_function(start_node_id);

    open_queue.push_back(start_node_id);
    open_queue_set.insert(start_node_id);
    arrival_time[start_node_id].value = start_time;

    auto handle_successor = [&](int current_id, int successor_id, double current_time, const TemporalGraphEdge& edge, double edge_time) {
        if (closed_list_set.find(successor_id) != closed_list_set.end()) {
            return;
        }

        tprm::TemporalGraphNode current_node = getNode(current_id);
        tprm::TemporalGraphNode next_node = getNode(successor_id);
        if ( ! interpolation_collision_check(current_node.position, next_node.position, current_time, edge_time, obstacles) )
            return;

        double tentative_g_cost = g_costs[current_id].value + getEdgeCost(edge);

        auto open_queue_set_it = open_queue_set.find(successor_id);

        if (open_queue_set_it != open_queue_set.end()) {
            if (tentative_g_cost >= g_costs[successor_id].value) {
                return;
            }
        }

        predecessor_map[successor_id] = current_id;
        g_costs[successor_id].value = tentative_g_cost;
        f_costs[successor_id].value = g_costs[successor_id].value + heuristic_function(successor_id);
        arrival_time[successor_id].value = current_time + edge_time;

        if (open_queue_set_it == open_queue_set.end()) {
            open_queue.push_back(successor_id);
            open_queue_set.insert(successor_id);
        }
    };

    while (!open_queue.empty()) {
        int current_id = open_queue.front();
        open_queue.pop_front();
        open_queue_set.erase(current_id);

        if (current_id == end_node_id) {
            break;
        }

        closed_list_set.insert(current_id);
        for (const TemporalGraphEdge& edge : edge_buckets[current_id]) {
            int successor_id = edge.getOtherNodeId(current_id);

            double current_time = arrival_time[current_id].value;
            double edge_time = (getNode(current_id).position - getNode(successor_id).position).norm() / HolonomicRobot::movement_speed;

            // check TA
            if (m_nodes[successor_id].isActiveAt(current_time + edge_time)) {
                handle_successor(current_id, successor_id, current_time, edge, edge_time);
            }
        }

        std::sort(open_queue.begin(), open_queue.end(), compare);
    }

    // check if goal is reached (arrival time < INF)
    if (arrival_time[end_node_id].value == std::numeric_limits<double>::infinity()) {
        return std::vector<GraphPathResultEntry>();
    }

    // reconstruct path by backtracking
    std::vector<GraphPathResultEntry> path;
    int current_id = end_node_id;
    while (current_id != start_node_id) {
        path.push_back(GraphPathResultEntry(current_id, arrival_time[current_id].value));
        current_id = predecessor_map[current_id];
    }
    path.push_back(GraphPathResultEntry(current_id, arrival_time[current_id].value));

    std::reverse(path.begin(), path.end());
    return path;
}

// returns false if a collision occurs
// returns true if no collision occurs
bool TemporalGraph::interpolation_collision_check(tprm::VectorNd current_position, tprm::VectorNd next_position, double current_time, double edge_time, 
                                                  const std::vector<std::shared_ptr<tprm::DynamicObstacle>>& obstacles) const
{
    /////////////////////////////////////
    // Interpolate between start node and goal node
    int interpolation_amount = 100;
    double deltaT = edge_time;

    for (int i = 0; i < interpolation_amount; i++)
    {
        
        //t = (time - t1) / (t2 - t1)
        //x = x1 + t * (x2 - x1)
        //y = y1 + t * (y2 - y1)
        //z = z1 + t * (z2 - z1)

        float percent = (float)i / interpolation_amount;

        tprm::VectorNd pos_interpol(0, dim);
        double time_interpol;
        for (int j = 0; j < dim; j++)
        {
            pos_interpol(j, current_position(j) + percent * (next_position(j) - current_position(j)) );
        }
        time_interpol = current_time + percent * deltaT;

        // 2. Update g_planning_state with obstacles and the state of the robot
        // Update obstacles and robot
        update_planning_scene_robot(pos_interpol.convertToStdVect());

        for (const auto& obstacle : obstacles) {
            update_planning_scene_obstacles(obstacle->getCOM(time_interpol), obstacle->m_radius, obstacle->m_id);
        }

        // 3. Run collision check function
        if ( !checkForCollisions() )
        {
            return false;
        }
        
    }

    return true;
    /////////////////////////////////////
}

void TemporalGraph::update_planning_scene_robot(std::vector<double> joint_values) const
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

void TemporalGraph::update_planning_scene_obstacles(Eigen::Vector3d position, float r, int id) const
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

bool TemporalGraph::checkForCollisions() const
{
    // Checking for Collisions
    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.contacts = true;
    c_req.max_contacts = 100;
    c_req.max_contacts_per_pair = 5;
    c_req.verbose = false;
    g_planning_scene->checkCollision(c_req, c_res, *kinematic_state);
    
    //if (c_res.contact_count > 0)
    //    ROS_INFO_STREAM("COLLIDING contact_point_count: " << c_res.contact_count);

    return c_res.contact_count == 0;
}

void TemporalGraph::setEdgeCostFunction(std::function<double(const TemporalGraphEdge&)> edge_cost_function) {
    m_edge_cost_function = edge_cost_function;
}

double TemporalGraph::defaultEdgeCostFunction(const TemporalGraphEdge& edge) const {
    return (getNode(edge.node_a_id).position - getNode(edge.node_b_id).position).norm();
};

double TemporalGraph::getEdgeCost(const TemporalGraphEdge& edge) const {
    return m_edge_cost_function(edge);
}

} /* namespace tprm */