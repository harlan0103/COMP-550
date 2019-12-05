///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 5
// Authors: Haoran Liang(hl74) & Hao Ding(hd25)
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include <ompl/control/SpaceInformation.h>

// State space
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Simple setup
#include <ompl/geometric/SimpleSetup.h>

// Planners
#include <ompl/geometric/planners/prm/PRM.h>
#include "dRRT.h"

// The collision checker produced in project 2
#include "CollisionChecking.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
using namespace std::placeholders;

// Define Robot structure
struct Robot
{
    // Define start and goal states for robot
    double startX;
    double startY;

    double goalX;
    double goalY;
};

// This is our state validity checker for checking if our square robot is in collision
bool isValidStateSquare(const ompl::base::State *state, double sideLength, const std::vector<Rectangle> &obstacles) 
{
    // Cast the state to a compound state
    auto cstate = state->as<ompl::base::CompoundState>();

    // Extract the real vector component (x, y)
    auto r2state = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
    double x = r2state->values[0];
    double y = r2state->values[1];

    // Extract theta (SO(2))
    auto so2State = cstate->as<ompl::base::SO2StateSpace::StateType>(1);
    double theta = so2State->value;

    return isValidSquare(x, y, theta, sideLength, obstacles);
}

void makeEnvironment1(std::vector<Rectangle> & obstacles)
{
    // TODO: Fill in the vector of rectangles with your street environment.

    // Create 4 obstacles as street
    Rectangle center;
    center.x = -2;
    center.y = -2;
    center.width = 4;
    center.height = 4;

    Rectangle top;
    top.x = -10;
    top.y = 7;
    top.width = 20;
    top.height = 3;

    Rectangle bottom;
    bottom.x = -10;
    bottom.y = -10;
    bottom.width = 20;
    bottom.height = 3;

    Rectangle left;
    left.x = -10;
    left.y = -3;
    left.width = 5;
    left.height = 6;

    Rectangle right;
    right.x = 5;
    right.y = -3;
    right.width = 5;
    right.height = 6;

    obstacles.push_back(center);
    obstacles.push_back(top);
    obstacles.push_back(bottom);
    obstacles.push_back(left);
    obstacles.push_back(right);
}

void makeEnvironment2(std::vector<Rectangle> & obstacles)
{
    // TODO: Fill in the vector of rectangles with your street environment.

    // Create 4 obstacles as street
    Rectangle center;
    center.x = -10;
    center.y = -2;
    center.width = 6;
    center.height = 4;

    Rectangle top;
    top.x = -2;
    top.y = 4;
    top.width = 4;
    top.height = 6;

    Rectangle bottom;
    bottom.x = 4;
    bottom.y = -2;
    bottom.width = 6;
    bottom.height = 4;

    Rectangle left;
    left.x = -2;
    left.y = -10;
    left.width = 4;
    left.height = 6;

    Rectangle right;
    right.x = -1;
    right.y = -1;
    right.width = 2;
    right.height = 2;

    obstacles.push_back(center);
    obstacles.push_back(top);
    obstacles.push_back(bottom);
    obstacles.push_back(left);
    obstacles.push_back(right);
}

void planMultipleRobots(std::vector<Rectangle> & obstacles)
{
    // TODO: Setup plan for multi-robots
    std::cout << "Set up plan for multi-robots" << std::endl;

    // Create state space for the system
    ob::StateSpacePtr se2;

    // Create R^2 component of the space
    auto r2 = std::make_shared<ob::RealVectorStateSpace>(2);

    // Set the bound for R^2
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);

    // Set the bounds on R^2
    r2->setBounds(bounds);

    // Create the SO(2) component of the state space
    auto so2 = std::make_shared<ob::SO2StateSpace>();

    // Create the compound state space
    se2 = r2 + so2;

    // Create simple setup container
    ompl::geometric::SimpleSetup ss(se2);

    // Setup the StateValidityChecker
    ss.setStateValidityChecker(std::bind(isValidStateSquare, _1, 1, obstacles));

    // Specify a planning algorithm to use
    auto planner = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // We have 4 robots and each has its own start and goal states

    /* env1 starts and goals */
    // Robot rbt1;
    // rbt1.startX = -8;
    // rbt1.startY = 5;
    // rbt1.goalX = 4;
    // rbt1.goalY = -6;

    // Robot rbt2;
    // rbt2.startX = 8;
    // rbt2.startY = 5;
    // rbt2.goalX = -4;
    // rbt2.goalY = -6;

    // Robot rbt3;
    // rbt3.startX = -8;
    // rbt3.startY = -5;
    // rbt3.goalX = 9;
    // rbt3.goalY = 6;

    // Robot rbt4;
    // rbt4.startX = 8;
    // rbt4.startY = -5;
    // rbt4.goalX = -9;
    // rbt4.goalY = 6;

    /* env2 starts and goals */
    Robot rbt1;
    rbt1.startX = -5;
    rbt1.startY = 5;
    rbt1.goalX = 5;
    rbt1.goalY = 5;

    Robot rbt2;
    rbt2.startX = 5;
    rbt2.startY = 5;
    rbt2.goalX = 5;
    rbt2.goalY = -5;

    Robot rbt3;
    rbt3.startX = 5;
    rbt3.startY = -5;
    rbt3.goalX = -5;
    rbt3.goalY = -5;

    Robot rbt4;
    rbt4.startX = -5;
    rbt4.startY = -5;
    rbt4.goalX = -5;
    rbt4.goalY = 5;

    // Specify the start and goal states for 1st robot

    std::cout << "robot1" << std::endl;
    ob::ScopedState<> r1Start(se2);
    r1Start[0] = rbt1.startX;
    r1Start[1] = rbt1.startY;
    
    ob::ScopedState<> r1goal(se2);
    r1goal[0] = rbt1.goalX;
    r1goal[1] = rbt1.goalY;

    // set the start and goal states
    ss.setStartAndGoalStates(r1Start, r1goal);

    // Attempt to solve the problem within the givin time
    ss.solve(1.0);
    planner->clearQuery();

    // Robot2 
    std::cout << "robot2" << std::endl;
    ob::ScopedState<> r2Start(se2);
    r2Start[0] = rbt2.startX;
    r2Start[1] = rbt2.startY;
    
    ob::ScopedState<> r2goal(se2);
    r2goal[0] = rbt2.goalX;
    r2goal[1] = rbt2.goalY;

    // set the start and goal states
    ss.setStartAndGoalStates(r2Start, r2goal);

    // Attempt to solve the problem within the givin time
    ss.solve(1.0);
    planner->clearQuery();

    // Robot3
    std::cout << "robot3" << std::endl;
    ob::ScopedState<> r3Start(se2);
    r3Start[0] = rbt3.startX;
    r3Start[1] = rbt3.startY;
    
    ob::ScopedState<> r3goal(se2);
    r3goal[0] = rbt3.goalX;
    r3goal[1] = rbt3.goalY;

    // set the start and goal states
    ss.setStartAndGoalStates(r3Start, r3goal);

    // Attempt to solve the problem within the givin time
    ss.solve(1.0);
    planner->clearQuery();

    // Robot4
    std::cout << "robot4" << std::endl;
    ob::ScopedState<> r4Start(se2);
    r4Start[0] = rbt4.startX;
    r4Start[1] = rbt4.startY;
    
    ob::ScopedState<> r4goal(se2);
    r4goal[0] = rbt4.goalX;
    r4goal[1] = rbt4.goalY;

    // set the start and goal states
    ss.setStartAndGoalStates(r4Start, r4goal);

    // Attempt to solve the problem within the givin time
    ob::PlannerStatus r4Solved = ss.solve(1.0);
    //ss.solve(1.0);

    // Try to get planner generated vertex and edges
    auto plannerData = std::make_shared<ob::PlannerData>(ss.getSpaceInformation());
    planner->getPlannerData(*plannerData);

    // Create composite roadmap
    std::vector<const ob::State*> roadMap;

    for(int i = 0; i < plannerData->numVertices(); i++) {
        const ob::State *st = plannerData->getVertex(i).getState();
        // Add vertex into roadMap
        roadMap.push_back(st);

        const ob::CompoundStateSpace::StateType& cs = *st->as<ob::CompoundStateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType& pos = *cs.as<ob::RealVectorStateSpace::StateType>(0);
        std::cout << pos[0] << " " << pos[1] << std::endl;
    }                

    // We need create composite state space
    std::cout << "Creat composite state space" << std::endl;
    // Create state space for the system
    ob::StateSpacePtr se8;

    // Create R^2 component of the space
    auto r8 = std::make_shared<ob::RealVectorStateSpace>(8);

    // Set the bound for R^2
    ob::RealVectorBounds compositeBounds(8);
    compositeBounds.setLow(-10);
    compositeBounds.setHigh(10);

    // Set the bounds on R^2
    r8->setBounds(compositeBounds);

    // Create the SO(2) component for each robot
    auto so2R1 = std::make_shared<ob::SO2StateSpace>();
    auto so2R2 = std::make_shared<ob::SO2StateSpace>();
    auto so2R3 = std::make_shared<ob::SO2StateSpace>();
    auto so2R4 = std::make_shared<ob::SO2StateSpace>();

    // Create the compound state space
    se8 = r8 + so2R1 + so2R2 + so2R3 + so2R4;

    // Create simple setup container
    ompl::geometric::SimpleSetup robotSetup(se8);

    // Setup the StateValidityChecker
    robotSetup.setStateValidityChecker(std::bind(isValidStateSquare, _1, 1, obstacles));

    // Set start and goal state for composite states
    ob::ScopedState<> compositeStart(se8);
    compositeStart[0] = rbt1.startX;
    compositeStart[1] = rbt1.startY;
    compositeStart[2] = rbt2.startX;
    compositeStart[3] = rbt2.startY;
    compositeStart[4] = rbt3.startX;
    compositeStart[5] = rbt3.startY;
    compositeStart[6] = rbt4.startX;
    compositeStart[7] = rbt4.startY;
    
    ob::ScopedState<> compositeGoal(se8);
    compositeGoal[0] = rbt1.goalX;
    compositeGoal[1] = rbt1.goalY;
    compositeGoal[2] = rbt2.goalX;
    compositeGoal[3] = rbt2.goalY;
    compositeGoal[4] = rbt3.goalX;
    compositeGoal[5] = rbt3.goalY;
    compositeGoal[6] = rbt4.goalX;
    compositeGoal[7] = rbt4.goalY;

    // set the start and goal states
    robotSetup.setStartAndGoalStates(compositeStart, compositeGoal);

    // Specify a planning algorithm to use
    auto robotPlanner = std::make_shared<ompl::geometric::dRRT>(robotSetup.getSpaceInformation());
    robotPlanner->getRoadMap(roadMap);
    robotPlanner->getRoadMapData(plannerData);

    robotSetup.setPlanner(robotPlanner);

    // Attempt to solve the problem within the givin time
    ob::PlannerStatus solved = robotSetup.solve(30.0);

    if(solved) {
        // Print path
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric &path = robotSetup.getSolutionPath();
        path.printAsMatrix(std::cout);
    }
    else {
        std::cout << "Not solved" << std::endl;
    }
}

void planSingleRobot(const std::vector<Rectangle> &obstacles)
{
    // Step 1) Create the state (configuration) space for your system
    ompl::base::StateSpacePtr se2;

    // Create the R^2 component of the space
    auto r2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);

    // We need to set bounds on R^2
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10);  // x and y have a minimum of -2
    bounds.setHigh(10);  // x and y have a maximum of 2

    // Set the bounds on R^2
    r2->setBounds(bounds);

    // Create the SO(2) component of the state space
    auto so2 = std::make_shared<ompl::base::SO2StateSpace>();

    // Create the compound state space (R^2 X SO(2) = SE(2))
    se2 = r2 + so2;

    // Step 2) Create the SimpleSetup container for the motion planning problem.
    ompl::geometric::SimpleSetup ss(se2);

    // Step 3) Setup the StateValidityChecker
    ss.setStateValidityChecker(std::bind(isValidStateSquare, _1, 0.3, obstacles));

    // Step 4) Specify the start and goal states
    ompl::base::ScopedState<> start(se2);
    start[0] = -8.0;
    start[1] = 5.0;
    start[2] = 0.0;

    ompl::base::ScopedState<> goal(se2);
    goal[0] = 4.0;
    goal[1] = -6.0;
    goal[2] = 0.0;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // Step 5) (optional) Specify a planning algorithm to use
    auto planner = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // Step 6) Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(20.0);

    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
        ss.simplifySolution();

        // print the path to screen
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric &path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

        // print path to file
        std::ofstream fout("path.txt");
        fout << "SE2" << std::endl;
        path.printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}


int main(int /* argc */, char ** /* argv */)
{
    // Make environment
    std::vector<Rectangle> obstacles;

    int env;
    do
    {
        std::cout << "Select an environment" << std::endl;
        std::cout << " (1) env1" << std::endl;
        std::cout << " (2) env2" << std::endl;

        std::cin >> env;
    } while (env != 1 && env != 2);
    
    if (env == 1)
    {
        makeEnvironment1(obstacles);
    }
    else if (env == 2)
    {
        makeEnvironment2(obstacles);
    }
    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;
    

    int choice;
    do
    {
        std::cout << "Select a plan" << std::endl;
        std::cout << " (1) Multi-robots using dRRT" << std::endl;
        std::cout << " (2) Single robot using PRM" << std::endl;

        std::cin >> choice;
    } while (choice != 1 && choice != 2);

    // Planning
    if (choice == 1)
    {
        planMultipleRobots(obstacles);
    }
    else if (choice == 2)
    {
        planSingleRobot(obstacles);
    }
    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
