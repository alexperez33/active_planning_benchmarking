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

#ifndef __TPRM_TEMPORAL_PRM_H__
#define __TPRM_TEMPORAL_PRM_H__

#include <tprm/obstacle_base.h>
#include <tprm/temporal_graph.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <iostream>
#include <memory>

namespace tprm {

/**
 * @brief Class to represent a path entry for the found path.
 * 
 * An entry consists of a position and the time at which the position is reached.
 */
struct PathResultEntry {
    /**
     * @brief Constructor.
     * @param position The position of the entry.
     * @param time The time at which the position is reached.
     */
    PathResultEntry(VectorNd position, double time) : position(position), time(time) {}
    VectorNd position;  ///< The position of the entry.
    double time;        ///< The time at which the position is reached.
};

/**
 * @brief Class to represent the T-PRM PRM.
 */
class TemporalPRM {
public:
    /**
     * @brief Constructor.
     */
    TemporalPRM() = default;

    TemporalPRM(planning_scene::PlanningScene* _g_planning_scene, 
                moveit::core::RobotModelConstPtr _kinematic_model, 
                moveit::core::RobotState* _kinematic_state,
                uint dimensions,
                std::string _robot_group,
                bool _dual_arm);

    /**
     * @brief Constructor.
     * @param environmentMin The minimum coordinates of the environment.
     * @param environmentMax The maximum coordinates of the environment.
     */
    TemporalPRM(const VectorNd& environmentMin, const VectorNd& environmentMax);

    /**
     * @brief Destructor.
     */
    virtual ~TemporalPRM() = default;

    /**
     * @brief Set the minimum coordinates of the environment.
     * @param environmentMin The minimum coordinates of the environment.
     */
    void setEnvironmentMin(const VectorNd& environmentMin);

    /**
     * @brief Set the maximum coordinates of the environment.
     * @param environmentMax The maximum coordinates of the environment.
     */
    void setEnvironmentMax(const VectorNd& environmentMax);

    /**
     * @brief Add a static obstacle to the environment.
     * @param obstacle The obstacle to add.
     */
    void addStaticObstacle(const std::shared_ptr<StaticObstacle>& obstacle);

    /**
     * @brief Add a dynamic obstacle to the environment.
     * @param obstacle The obstacle to add.
     */
    void addDynamicObstacle(const std::shared_ptr<DynamicObstacle>& obstacle);

    /**
     * @brief Place sample points on the environment.
     * 
     * @attention Static obstacles must be added before calling this!
     * @param numSamples The number of sample points to place.
     */
    void placeSamples(int numSamples);

    int addSample(VectorNd _sample, double costEdgeThreshold);

    /**
     * @brief Connect the sample points.
     * 
     * @attention Sample points must be placed before calling this!
     * @attention Dynamic obstacles must be added before calling this! (otherwise, call TemporalPRM::recomputeTimeAvailabilities)
     * @param costEdgeThreshold The cost threshold for the edges.
     */
    void buildPRM(double costEdgeThreshold);

    /**
     * @brief Recompute the time availabilities of the nodes in the graph.
     * 
     * @attention Dynamic obstacles must be added before calling this!
     */
    void recomputeTimeAvailabilities();

    /**
     * @brief Get the underlying graph.
     * @return The underlying graph.
     */
    TemporalGraph& getTemporalGraph();

    /**
     * @brief Find the shortest path between two points.
     * @param start The start position.
     * @param goal The goal position.
     * @param timeStart The time at which to start movement at the start position.
     * @return The found path (positions and respective arrival times).
     */
    std::vector<PathResultEntry> getShortestPath(const VectorNd& start, const VectorNd& goal, double timeStart = 0.0) const;

    std::vector<PathResultEntry> getShortestPath(const int start, const int goal, double timeStart) const;

private:
    /**
     * @brief Compute time availability for a position.
     * @param position The position.
     * @return The time availability (set of intervals).
     */
    std::vector<TimeAvailability> getTimeAvailability(const VectorNd& position) const;

    void update_planning_scene_robot(std::vector<double> joint_values) const;

    void update_planning_scene_obstacles(Eigen::Vector3d position, float r, int id) const;

    bool checkForCollisions() const;

    std::vector<TimeAvailability> discreteAvailableToTimeInvervals(std::vector<double> discreteAvailabilities, double time_increment, double time_limit) const;

    TemporalGraph m_graph;  ///< The underlying graph.

    uint dim = 2; // The number of dimensions the graph has
    bool dual_arm = false;
    std::string robot_group;
    VectorNd m_environmentMin;  ///< The minimum coordinates of the environment.
    VectorNd m_environmentMax;  ///< The maximum coordinates of the environment.

    std::vector<std::shared_ptr<StaticObstacle>> m_staticObstacles;    ///< The static obstacles.
    std::vector<std::shared_ptr<DynamicObstacle>> m_dynamicObstacles;  ///< The dynamic obstacles.

    planning_scene::PlanningScene* g_planning_scene;
    moveit::core::RobotModelConstPtr kinematic_model;
    moveit::core::RobotState* kinematic_state;
};

} /* namespace tprm */

#endif /* __TPRM_TEMPORAL_PRM_H__ */