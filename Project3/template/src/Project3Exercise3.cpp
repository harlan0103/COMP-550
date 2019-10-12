///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Hao Ding (hd25) & Haoran Liang (hl74)
//////////////////////////////////////

#include <iostream>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>

#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>

// Your random tree planner
#include "RTP.h"
using namespace ompl;


void benchmarkCubicles(std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    // TODO
    // Sample codes from SE3RigidBodyPlanningBenchmark.cpp
    benchmark_name = std::string("cubicles");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(-4.96);
    start->setY(-40.62);
    start->setZ(70.57);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(200.49);
    goal->setY(-40.62);
    goal->setZ(70.57);
    goal->rotation().setIdentity();

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    setup.setup();

    std::vector<double> cs(3);
    cs[0] = 35;
    cs[1] = 35;
    cs[2] = 35;
    setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);

    runtime_limit = 50.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 50;	// Compare planners in 50 independent runs

}

void benchmarkTwistycool(std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    // TODO
    // Sample codes from SE3RigidBodyPlanningBenchmark.cpp
    benchmark_name = std::string("Twistycool");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(270.);
    start->setY(160.);
    start->setZ(-200.);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(270.);
    goal->setY(160.);
    goal->setZ(-400.);
    goal->rotation().setIdentity();

    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 350.);
    bounds.setHigh(1, 250.);
    bounds.setHigh(2, -150.);
    bounds.setLow(0, 200.);
    bounds.setLow(1, 75.);
    bounds.setLow(2, -450.);
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    runtime_limit = 100.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 50;
}

int main(int argc, char ** argv)
{
    int environment;
	app::SE3RigidBodyPlanning setup;
    std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;

    do
    {
        std::cout << "Benchmark for: " << std::endl;
        std::cout << " (1) Cubicles" << std::endl;
        std::cout << " (2) Twistycool" << std::endl;

        std::cin >> environment;
    } while (environment < 1 || environment > 2);

    switch (environment)
    {
        case 1:
            benchmarkCubicles(benchmark_name, setup, runtime_limit, memory_limit, run_count);
            break;
        case 2:
            benchmarkTwistycool(benchmark_name, setup, runtime_limit, memory_limit, run_count);
            break;
        default:
            std::cerr << "Invalid Environment!" << std::endl;
            break;
    }

    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    tools::Benchmark b(setup, benchmark_name);

    b.addPlanner(std::make_shared<geometric::RRT>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::EST>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::RTP>(setup.getSpaceInformation()));


    // run all planners with a uniform valid state sampler on the benchmark problem
    setup.getSpaceInformation()->setValidStateSamplerAllocator(
        [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr {
            return std::make_shared<base::UniformValidStateSampler>(si);
        });
    b.addExperimentParameter("sampler_id", "INTEGER", "0");
    b.benchmark(request);

    if(environment == 1) {
		b.saveResultsToFile("benchmark_cubicles.log");
    }
    else {
    	b.saveResultsToFile("benchmark_twistycool.log");
    }
    
    return 0;
}
