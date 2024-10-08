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

#include <benchmarking/OMPLPlanner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/STRRTstar.h>
#include <fstream>
#include <chrono>
using namespace std::chrono;

//#define OMPL_USE_NUM_NODES
#define OMPL_USE_EDGE_LENGTH
namespace ob = ompl::base;

namespace ompl::geometric {

class MyPRM : public PRM {
public:
    MyPRM(ompl::base::SpaceInformationPtr si, double edgeThreshold) : PRM(si), m_edgeThreshold(edgeThreshold) {
        setConnectionFilter(std::bind(&MyPRM::connectionFilter, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    bool connectionFilter(const Vertex& v1, const Vertex& v2) const {
        return PRM::distanceFunction(v1, v2) < m_edgeThreshold;
    }
    double m_edgeThreshold;
};
}  // namespace ompl::geometric

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
        if (!(deltaT > 0 && deltaPos / deltaT <= vMax_+0.01)) {
            invalid_++;
            return false;
        }

        // check if the path between the states is unconstrained (perform interpolation)...
        int interpolation_amount = 25;
        double posx1 = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double posy1 = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        double posz1 = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[2];
        double posx2 = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double posy2 = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        double posz2 = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[2];
        double posx0; //Variable
        double posy0; //Variable
        double posz0; //Variable
        double time0; //Variable

        for (int i = 0; i < interpolation_amount; i++)
        {
            /*
            t = (time - t1) / (t2 - t1)
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            z = z1 + t * (z2 - z1)
            */

            float percent = (float)i / interpolation_amount;
            posx0 = posx1 + percent * (posx2 - posx1);
            posy0 = posy1 + percent * (posy2 - posy1);
            posz0 = posz1 + percent * (posz2 - posz1);
            time0 = time1 + percent * deltaT;

            auto space = si_->getStateSpace();
            ompl::base::ScopedState<> temp_state(space);
            temp_state->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = posx0;
            temp_state->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = posy0;
            temp_state->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = posz0;
            temp_state->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ob::TimeStateSpace::StateType>(1)->position = time0;

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

namespace benchmarking {

class ValidityChecker : public ompl::base::StateValidityChecker {
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr& si, std::shared_ptr<benchmarking::Benchmark> bm) : ompl::base::StateValidityChecker(si), benchmark(bm) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ompl::base::State* state) const {
        return this->clearance(state) > 0.0;
    }

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ompl::base::State* state) const {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ompl::base::RealVectorStateSpace::StateType* state2D = state->as<ompl::base::RealVectorStateSpace::StateType>();

        double min_dist = 1e4;
        if (benchmark->is_2d) {
            double x = state2D->values[0];
            double y = state2D->values[1];
            Eigen::Vector2d pos(x, y);
            double min_dist = 1e4;
            for (const benchmarking::Circle& c : benchmark->circles) {
                // get minimum distance to circle
                double dist = (pos - c.center.head(2)).norm() - c.radius;
                if (dist < min_dist)
                    min_dist = dist;
            }
        } else {  // 3D case
            // Extract the robot's (x,y) position from its state
            double x = state2D->values[0];
            double y = state2D->values[1];
            double z = state2D->values[2];
            Eigen::Vector3d pos(x, y, z);

            // static obstacles:
            double min_dist = 1e4;
            for (const benchmarking::Circle& c : benchmark->circles) {
                // get minimum distance to circle
                double dist = (pos - c.center).norm() - c.radius;
                if (dist < min_dist)
                    min_dist = dist;
            }
        }

        return min_dist;
    }

private:
    std::shared_ptr<benchmarking::Benchmark> benchmark;
};

class ValidityCheckerDynObst : public ompl::base::StateValidityChecker {
public:
    ValidityCheckerDynObst(const ompl::base::SpaceInformationPtr& si, std::shared_ptr<benchmarking::Benchmark> bm, double time)
        : ompl::base::StateValidityChecker(si), benchmark(bm), m_time(time) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ompl::base::State* state) const {
        return this->clearance(state) > 0.0;
    }

    void set_time(double time) {
        m_time = time;
    }

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ompl::base::State* state) const {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ompl::base::RealVectorStateSpace::StateType* state2D = state->as<ompl::base::RealVectorStateSpace::StateType>();

        double min_dist = 1e4;
        if (benchmark->is_2d) {
            // Extract the robot's (x,y) position from its state
            double x = state2D->values[0];
            double y = state2D->values[1];
            Eigen::Vector2d pos(x, y);

            // static obstacles:

            for (const benchmarking::Circle& c : benchmark->circles) {
                // get minimum distance to circle
                double dist = (pos - c.center.head(2)).norm() - c.radius;
                if (dist < min_dist)
                    min_dist = dist;
            }

            for (const benchmarking::MovingCircle& mc : benchmark->moving_circles) {
                Eigen::Vector3d pos_mc = mc.center + mc.velocity * m_time;
                double dist = (pos - pos_mc.head(2)).norm() - mc.radius;
                if (dist < min_dist)
                    min_dist = dist;
            }
        } else {  // 3D
            // Extract the robot's (x,y) position from its state
            double x = state2D->values[0];
            double y = state2D->values[1];
            double z = state2D->values[2];
            Eigen::Vector3d pos(x, y, z);

            // static obstacles:

            for (const benchmarking::Circle& c : benchmark->circles) {
                // get minimum distance to circle
                double dist = (pos - c.center).norm() - c.radius;
                if (dist < min_dist)
                    min_dist = dist;
            }

            for (const benchmarking::MovingCircle& mc : benchmark->moving_circles) {
                Eigen::Vector3d pos_mc = mc.center + mc.velocity * m_time;
                double dist = (pos - pos_mc).norm() - mc.radius;
                if (dist < min_dist)
                    min_dist = dist;
            }
        }

        return min_dist;
    }

private:
    std::shared_ptr<benchmarking::Benchmark> benchmark;
    double m_time;
};

class STValidityCheckerDynObst : public ompl::base::StateValidityChecker {
public:
    STValidityCheckerDynObst(const ompl::base::SpaceInformationPtr& si, std::shared_ptr<benchmarking::Benchmark> bm, double time)
        : ompl::base::StateValidityChecker(si), benchmark(bm), m_time(time) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ompl::base::State* state) const {
        bool valid = this->clearance(state) > 0.0;
        return valid;
    }

    void set_time(double time) {
        m_time = time;
    }

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ompl::base::State* state) const {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ompl::base::RealVectorStateSpace::StateType* state2D = 
            state->as<ob::SpaceTimeStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0);

        double min_dist = 1e4;
        if (benchmark->is_2d) {
            // Extract the robot's (x,y) position from its state
            double x = state2D->values[0];
            double y = state2D->values[1];
            Eigen::Vector2d pos(x, y);

            // static obstacles:

            for (const benchmarking::Circle& c : benchmark->circles) {
                std::cout << "stagnant circles" << std::endl;
                
                // get minimum distance to circle
                double dist = (pos - c.center.head(2)).norm() - c.radius;
                if (dist < min_dist)
                    min_dist = dist;
            }

            int i = 0;
            for (const benchmarking::MovingCircle& mc : benchmark->moving_circles) {

                Eigen::Vector3d pos_mc;
                try{
                    double time = state->as<ob::SpaceTimeStateSpace::StateType>()->as<ob::TimeStateSpace::StateType>(1)->position;
                    pos_mc = mc.center + mc.velocity * time;
                } catch(...)
                {
                    pos_mc = mc.center + mc.velocity * m_time;
                }

                double dist = (pos - pos_mc.head(2)).norm() - mc.radius;

                //std::cout << i << ": " << dist << ": " << time << std::endl;
                if (dist < min_dist)
                    min_dist = dist;
                
                i++;
            }
        } else {  // 3D
            // Extract the robot's (x,y) position from its state
            double x = state2D->values[0];
            double y = state2D->values[1];
            double z = state2D->values[2];
            Eigen::Vector3d pos(x, y, z);

            // static obstacles:

            for (const benchmarking::Circle& c : benchmark->circles) {
                // get minimum distance to circle
                double dist = (pos - c.center).norm() - c.radius;
                if (dist < min_dist)
                    min_dist = dist;
            }

            for (const benchmarking::MovingCircle& mc : benchmark->moving_circles) {
                Eigen::Vector3d pos_mc = mc.center + mc.velocity * m_time;
                double dist = (pos - pos_mc).norm() - mc.radius;
                if (dist < min_dist)
                    min_dist = dist;
            }
        }

        //std::cout << min_dist << std::endl;
        return min_dist;
    }

private:
    std::shared_ptr<benchmarking::Benchmark> benchmark;
    double m_time;
};


OMPLPlannerBenchmark::OMPLPlannerBenchmark(std::string planner) : m_planner(planner) {
    ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);
}

ompl::base::OptimizationObjectivePtr getThresholdPathLengthObj(const ompl::base::SpaceInformationPtr& si, double cost) {
    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ompl::base::Cost(cost));
    return obj;
}

void write_intermediate_path(std::shared_ptr<Benchmark> benchmark, const std::vector<Eigen::Vector3d>& path, double time, std::string name) {
    if (!benchmark->writeMovingIntermediatePaths)
        return;

    std::ofstream of("INTERMEDIATE_PATH_" + name + "_AT_" + std::to_string(time) + ".txt");
    if (path.size() == 0) {
        return;
    }
    for (int i = 0; i < path.size() - 1; i++) {
        of << path[i].x() << " " << path[i].y() << " " << path[i].z() << " " << path[i + 1].x() << " " << path[i + 1].y() << " " << path[i + 1].z()
           << std::endl;
    }
}

double strrt_goal_travel_time = 0;
bool strrt_solution_found = false;

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

bool strrt_terminating_condition(double solve_timeout, std::chrono::_V2::system_clock::time_point begin)
{

    // Check timeout condition
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<seconds>(stop - begin).count();
    double strrt_solve_time = duration;
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

BenchmarkResult OMPLPlannerBenchmark::runBenchmark(std::shared_ptr<Benchmark> benchmark, int benchmark_idx, int run_idx) {
    
    if (benchmark->moving_circles.size() == 0) {
        BasePlannerBenchmark::startBenchmark();

        BenchmarkResult result;

        ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2 + !benchmark->is_2d));
        space->as<ompl::base::RealVectorStateSpace>()->setBounds(0, benchmark->domain_size);

        ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

        ValidityChecker* validityChecker = new ValidityChecker(si, benchmark);

        si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(validityChecker));
        si->setup();

        ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
        pdef->setOptimizationObjective(getThresholdPathLengthObj(si, benchmark->ompl_path_length_threshold));

        ompl::base::PlannerPtr planner;
        if (m_planner == "RRTstar") {
            planner = std::make_shared<ompl::geometric::RRTstar>(si);
            dynamic_cast<ompl::geometric::RRTstar*>(planner.get())->setRange(benchmark->ompl_edge_length);
        } else if (m_planner == "PRM") {
#ifdef OMPL_USE_EDGE_LENGTH
            planner = std::make_shared<ompl::geometric::MyPRM>(si, benchmark->ompl_edge_length);
#else
            planner = std::make_shared<ompl::geometric::PRM>(si);
#endif
        } else if (m_planner != "") {
            std::cerr << "Unknown planner: " << m_planner << std::endl;
            exit(1);
        }
        ompl::base::ScopedState<> start(space);
        ompl::base::ScopedState<> goal(space);
        for (size_t query_idx = 0; query_idx < benchmark->start.size(); query_idx++) {
            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = benchmark->start[query_idx].x();
            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = benchmark->start[query_idx].y();
            if (!benchmark->is_2d)
                start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = benchmark->start[query_idx].z();

            goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = benchmark->goal[query_idx].x();
            goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = benchmark->goal[query_idx].y();
            if (!benchmark->is_2d)
                goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = benchmark->goal[query_idx].z();

            if (query_idx > 0) {
                pdef->clearSolutionPaths();
                pdef->clearStartStates();
                pdef->clearGoal();

                if (m_planner == "PRM") {
                    // cast planner
                    static_cast<ompl::geometric::PRM*>(planner.get())->clearQuery();
                } else if (

                    m_planner == "RRTstar") {
                    static_cast<ompl::geometric::RRTstar*>(planner.get())->clear();
                }
            }
            pdef->setStartAndGoalStates(start, goal);
            planner->setProblemDefinition(pdef);
            planner->setup();

            startMeasurement();
#ifndef OMPL_USE_NUM_NODES
            ompl::base::PlannerStatus status = planner->solve(benchmark->ompl_time_limit);
#else
            // if we want to limit the number of nodes: (hacky!)
            ompl::base::PlannerStatus status;
            if (m_planner == "PRM") {
                status = planner->solve(ompl::base::PlannerTerminationCondition([&]() -> bool {
                    // return true if we should stop
                    auto planner_ = static_cast<ompl::geometric::PRM*>(planner.get());
                    return planner_->milestoneCount() >= benchmark->numNodes;
                }));
            } else {
                status = planner->solve(ompl::base::PlannerTerminationCondition([&]() -> bool {
                    // return true if we should stop
                    auto planner_ = static_cast<ompl::geometric::RRTstar*>(planner.get());
                    return planner_->numIterations() >= benchmark->numNodes;
                }));
            }
#endif
            result.success.push_back(status == ompl::base::PlannerStatus::EXACT_SOLUTION);

            result.timing_results.push_back(stopMeasurement("Solve query " + std::to_string(query_idx)));

            ompl::base::PathPtr path = pdef->getSolutionPath();
            if (path != nullptr) {
                ompl::geometric::PathGeometric& pathGeometric = dynamic_cast<ompl::geometric::PathGeometric&>(*path);
                auto states = pathGeometric.getStates();
                result.path.push_back({});
                for (auto state : states) {
                    if (!benchmark->is_2d) {
                        result.path.back().push_back(Eigen::Vector3d(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                                                     state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
                                                                     state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]));
                    } else {  // pad with 0
                        result.path.back().push_back(Eigen::Vector3d(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                                                     state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1], 0.));
                    }
                }
            }
        }

        BasePlannerBenchmark::stopBenchmark();

        result.description = "OMPL " + m_planner + " - " + benchmark->name;
        result.duration_micros = getTotalDuration();

        return result;
    } else if (m_planner != "STRRTstar"){
        // moving circles --> run OMPL iteratively
        BasePlannerBenchmark::startBenchmark();

        BenchmarkResult result;

        ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2 + !benchmark->is_2d));
        space->as<ompl::base::RealVectorStateSpace>()->setBounds(0, benchmark->domain_size);

        ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

        ompl::base::StateValidityCheckerPtr validityChecker = std::make_shared<ValidityCheckerDynObst>(si, benchmark, 0.);
        si->setStateValidityChecker(validityChecker);

        si->setup();
        ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
        pdef->setOptimizationObjective(getThresholdPathLengthObj(si, benchmark->ompl_path_length_threshold));

        ompl::base::PlannerPtr planner;
        if (m_planner == "RRTstar") {
            planner = std::make_shared<ompl::geometric::RRTstar>(si);
            dynamic_cast<ompl::geometric::RRTstar*>(planner.get())->setRange(benchmark->ompl_edge_length);
        } else if (m_planner == "PRM") {
#ifdef OMPL_USE_EDGE_LENGTH
            planner = std::make_shared<ompl::geometric::MyPRM>(si, benchmark->ompl_edge_length);
#else
            planner = std::make_shared<ompl::geometric::PRM>(si);
#endif
        } else if (m_planner != "") {
            std::cerr << "Unknown planner: " << m_planner << std::endl;
            exit(1);
        }

        ompl::base::ScopedState<> start(space);
        ompl::base::ScopedState<> goal(space);
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = benchmark->start[0].x();
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = benchmark->start[0].y();
        if (!benchmark->is_2d)
            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = benchmark->start[0].z();

        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = benchmark->goal[0].x();
        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = benchmark->goal[0].y();
        if (!benchmark->is_2d)
            goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = benchmark->goal[0].z();

        pdef->setStartAndGoalStates(start, goal);
        planner->setProblemDefinition(pdef);
        planner->setup();

        startMeasurement();

        ompl::base::PlannerStatus status = planner->solve(benchmark->ompl_time_limit);
        result.success.push_back(status == ompl::base::PlannerStatus::EXACT_SOLUTION);

        result.timing_results.push_back(stopMeasurement("Solve query " + std::to_string(0)));

        ompl::base::PathPtr path = pdef->getSolutionPath();
        std::vector<Eigen::Vector3d> path_;
        if (path != nullptr) {
            ompl::geometric::PathGeometric& pathGeometric = dynamic_cast<ompl::geometric::PathGeometric&>(*path);
            auto states = pathGeometric.getStates();

            for (auto state : states) {
                if (!benchmark->is_2d) {
                    path_.push_back(Eigen::Vector3d(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                                    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
                                                    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]));
                } else {  // pad with 0
                    path_.push_back(Eigen::Vector3d(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                                    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1], 0.));
                }
            }
        }

        write_intermediate_path(benchmark, path_, 0., OMPLPlannerBenchmark::getName());

        double total_time = 0.;
        bool has_reached_end = false;
        std::vector<Eigen::Vector3d> full_path;
        bool in_collision = false;
        full_path.push_back(benchmark->start[0]);
        while (!has_reached_end) {
            in_collision = false;  // reset
            double time_on_current_path = 0.;

            if (path_.size() < 2) {
            /*std::cerr << "Path is too short (" << path_.size() << ")" << std::endl;
                for (auto& s : path_) {
                    std::cerr << s.transpose() << std::endl;
                }
                std::cerr << "Current start: " << start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] << " "
                          << start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] << " ";
                
                if (!benchmark->is_2d)
                    std::cerr << start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] << std::endl;
                */
                BasePlannerBenchmark::stopBenchmark();
                result.description = "OMPL " + m_planner + " - " + benchmark->name;
                result.duration_micros = getTotalDuration();
                result.success.back() = false;
                return result;
            }

            // search the current segment to use
            int current_segment = 0;
            //full_path.push_back(path_.at(current_segment));
            //std::cout << "Full path push back A: " << full_path.back().transpose() << std::endl;
            Eigen::Vector3d start_;
            double MOVE_FORWARD_TIME = 0.25;
            double time_on_segment = 0.;
            double speed = 0.5;
            while (!in_collision) {
                while (current_segment < path_.size() - 1) {
                    double time_for_segment = (path_.at(current_segment + 1) - path_.at(current_segment)).norm() / speed;

                    double move_time = std::min(MOVE_FORWARD_TIME, time_for_segment - time_on_segment);
                    time_on_current_path += move_time;
                    time_on_segment += move_time;

                    if (time_on_segment >= time_for_segment) {
                        // we have reached the end of the segment
                        current_segment++;
                        time_on_segment = 0.;
                        if (current_segment == path_.size() - 1) {
                            has_reached_end = true;
                            break;
                        }
                        full_path.push_back(path_.at(current_segment));
                        start_ = path_.at(current_segment);  // update start
                    } else {
                        // we are still on the same segment
                        start_ = path_.at(current_segment) + (path_.at(current_segment + 1) - path_.at(current_segment)) * (time_on_segment / time_for_segment);
                        full_path.push_back(start_);
                    }

                    // COLLISION CHECK
                    auto path = pdef->getSolutionPath();
                    if (path != nullptr) {
                        ompl::geometric::PathGeometric& pathGeometric = dynamic_cast<ompl::geometric::PathGeometric&>(*path);
                        auto states = pathGeometric.getStates();
                        ValidityCheckerDynObst val_checker(si, benchmark, total_time + time_on_current_path);
                        for (auto state : states) {
                            if (!val_checker.isValid(state)) {
                                in_collision = true;
                                // this means, we have to replan
                                break;
                            }
                        }

                        if (!in_collision) {
                            //std::cout << "Can continue with time " << time_on_current_path << " on segment " << current_segment << " with time "
                            //          << time_for_segment << std::endl;
                        } else {
                            //std::cout << "Collision detected at time " << time_on_current_path << std::endl;
                            break;
                        }
                    }
                }
                if (has_reached_end) {
                    // propagate break
                    break;
                }
            }
            if (has_reached_end) {
                // propagate break
                break;
            }

            //std::cout << "Replanning at time " << total_time + time_on_current_path << std::endl;

            static_cast<ValidityCheckerDynObst*>(validityChecker.get())->set_time(total_time + time_on_current_path);

            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = start_.x();
            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = start_.y();
            if (!benchmark->is_2d)
                start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = start_.z();

            // goal is the same

            si = std::make_shared<ompl::base::SpaceInformation>(space);
            si->setStateValidityChecker(validityChecker);
            si->setup();
            pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
            pdef->setOptimizationObjective(getThresholdPathLengthObj(si, benchmark->ompl_path_length_threshold));
            if (m_planner == "RRTstar") {
                planner = std::make_shared<ompl::geometric::RRTstar>(si);
                dynamic_cast<ompl::geometric::RRTstar*>(planner.get())->setRange(benchmark->ompl_edge_length);
            } else if (m_planner == "PRM") {
#ifdef OMPL_USE_EDGE_LENGTH
                planner = std::make_shared<ompl::geometric::MyPRM>(si, benchmark->ompl_edge_length);
#else

                planner = std::make_shared<ompl::geometric::PRM>(si);
#endif
            } else if (m_planner != "") {
                std::cerr << "Unknown planner: " << m_planner << std::endl;
                exit(1);
            }
            pdef->setStartAndGoalStates(start, goal);
            planner->setProblemDefinition(pdef);
            planner->setup();

            status = planner->solve(benchmark->ompl_time_limit);

            // and-ing success
            result.success.back() = result.success.back() & (status == ompl::base::PlannerStatus::EXACT_SOLUTION);
            path = pdef->getSolutionPath();
            path_.clear();
            if (path != nullptr) {
                ompl::geometric::PathGeometric& pathGeometric = dynamic_cast<ompl::geometric::PathGeometric&>(*path);
                auto states = pathGeometric.getStates();

                for (auto state : states) {
                    if (!benchmark->is_2d) {
                        path_.push_back(Eigen::Vector3d(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                                        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
                                                        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]));
                    } else {  // pad with 0.
                        path_.push_back(Eigen::Vector3d(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                                        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1], 0.));
                    }
                }
            }

            total_time += time_on_current_path;
            write_intermediate_path(benchmark, path_, total_time, OMPLPlannerBenchmark::getName());

            // reached end if start is close to goal 1e-3
            if (!benchmark->is_2d) {
                Eigen::Vector3d e_start(start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
                                        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]);
                Eigen::Vector3d e_goal(goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                       goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
                                       goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]);
                has_reached_end = (e_start - e_goal).norm() < 1e-3;
            } else {
                Eigen::Vector2d e_start(start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]);
                Eigen::Vector2d e_goal(goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                       goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]);
                has_reached_end = (e_start - e_goal).norm() < 1e-3;
            }
        }

        // push back goal
        full_path.push_back(benchmark->goal[0]);

        result.path.push_back(full_path);

        BasePlannerBenchmark::stopBenchmark();

        result.description = "OMPL " + m_planner + " - " + benchmark->name;
        result.duration_micros = getTotalDuration();

        return result;
    } else {
        BasePlannerBenchmark::startBenchmark();
        BenchmarkResult result;

        std::cout << "Entering strrt benchmark" << std::endl;
        // set maximum velocity
        // TODO: redefine vMax to be something from inputs
        double vMax = 0.5;
        strrt_goal_travel_time = 35;
        double max_solve_time = 5;

        //ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2 + !benchmark->is_2d));
        //space->as<ompl::base::RealVectorStateSpace>()->setBounds(0, benchmark->domain_size);
        // construct the state space we are planning in
        auto vectorSpace(std::make_shared<ob::RealVectorStateSpace>(2 + !benchmark->is_2d));
        auto space = std::make_shared<ob::SpaceTimeStateSpace>(vectorSpace, vMax);

        // set the bounds for R2 or R3
        ob::RealVectorBounds bounds(2 + !benchmark->is_2d);
        bounds.setLow(0);
        bounds.setHigh(benchmark->domain_size);
        vectorSpace->setBounds(bounds);

        // set time bounds. Planning with unbounded time is also possible when using ST-RRT*.
        // TODO: figure out how to make it unbounded
        space->setTimeBounds(0.0, 50.0);

        // create the space information class for the space
        ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);

        // set state validity checking for this space
        // TODO: check that motion validator works as expected
        // TODO: check that validity checker dyn obst works as expected with spacetime-based si
        ompl::base::StateValidityCheckerPtr validityChecker = std::make_shared<STValidityCheckerDynObst>(si, benchmark, 0.);
        si->setStateValidityChecker(validityChecker);
        //si->setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
        si->setMotionValidator(std::make_shared<SpaceTimeMotionValidator>(si));

        // define a simple setup class
        ompl::geometric::SimpleSetup ss(si);

        ompl::base::ScopedState<> start(space);
        ompl::base::ScopedState<> goal(space);
        start->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = benchmark->start[0].x();
        start->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = benchmark->start[0].y();
        if (!benchmark->is_2d)
            start->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = benchmark->start[0].z();

        goal->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = benchmark->goal[0].x();
        goal->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = benchmark->goal[0].y();
        if (!benchmark->is_2d)
            goal->as<ompl::base::SpaceTimeStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = benchmark->goal[0].z();

        // set the start and goal states
        ss.setStartAndGoalStates(start, goal);

        // construct the planner
        auto *strrtStar = new ompl::geometric::STRRTstar(si);

        // set planner parameters
        strrtStar->setRange(vMax);

        // set the used planner
        ss.setPlanner(ob::PlannerPtr(strrtStar));

        // terminating conditions
        auto begin = high_resolution_clock::now();
        std::function<bool()> terminating_function = [&]() { return strrt_terminating_condition(max_solve_time, begin); };
        ob::PlannerTerminationConditionFn fn(terminating_function);
        ob::PlannerTerminationCondition ptc(fn);

        std::function<void(const ob::Planner *, const std::vector<const ob::State *> &, const ob::Cost)> intermediate_function =
            [&](const ob::Planner * planner, const std::vector<const ob::State *> & states, const ob::Cost cost) 
                { return strrt_intermediate_condition(planner, states, cost); };
        ss.getProblemDefinition()->setIntermediateSolutionCallback(intermediate_function);

        // attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = ss.solve(ptc);
        std::cout << solved << std::endl;

        ompl::base::PlannerStatus::StatusType status;
        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            status = ompl::base::PlannerStatus::EXACT_SOLUTION;
            // print the path to screen
            //ss.getSolutionPath().print(std::cout);
        }
        else
            std::cout << "No solution found" << std::endl;

        //TODO: Add benchmark wrapper stuff around this so that we can visualize the solution
        ompl::geometric::PathGeometric fullpath = ss.getSolutionPath();
        std::vector<ompl::base::State*> states = fullpath.getStates();
        std::vector<Eigen::Vector3d> fullpath_;
        std::vector<double> fullpath_durations;
        for (ompl::base::State* state : states)
        {
            const ompl::base::SpaceTimeStateSpace::StateType* spaceTimeState =
                state->as<ompl::base::SpaceTimeStateSpace::StateType>();

            const ompl::base::RealVectorStateSpace::StateType* spatialState =
                spaceTimeState->as<ompl::base::RealVectorStateSpace::StateType>(0);
            
            double x = spatialState->values[0];
            double y = spatialState->values[1];
            double z = 0;
            if (!benchmark->is_2d)
                spatialState->values[2];
            double t = spaceTimeState->as<ob::TimeStateSpace::StateType>(1)->position;
            Eigen::Vector3d point_ = Eigen::Vector3d(x,y,z);

            fullpath_.push_back( point_ );
            fullpath_durations.push_back(t);
        }

        result.success.push_back(status == ompl::base::PlannerStatus::EXACT_SOLUTION);
        result.timing_results.push_back(stopMeasurement("Solve query " + std::to_string(0)));

        result.path.push_back(fullpath_);
        result.path_durations.push_back(fullpath_durations);
        BasePlannerBenchmark::stopBenchmark();

        result.description = "OMPL " + m_planner + " - " + benchmark->name;
        result.duration_micros = getTotalDuration();

        return result;
    }
}

}  // namespace benchmarking
