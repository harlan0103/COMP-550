/* Author: Haoran Liang(hl74) & Hao Ding(hd25) */

#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "dRRT.h"
#include "math.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include "CollisionChecking.h"

const double sideLen = 1.0;

ompl::geometric::dRRT::dRRT(const base::SpaceInformationPtr &si, bool addIntermediateStates)
    : base::Planner(si, addIntermediateStates ? "dRRTintermediate" : "dRRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;
    
    Planner::declareParam<double>("range", this, &dRRT::setRange, &dRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &dRRT::setGoalBias, &dRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &dRRT::setIntermediateStates, &dRRT::getIntermediateStates,
                                "0,1");
    
    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::dRRT::~dRRT()
{
    freeMemory();
}

void ompl::geometric::dRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
    roadMapData = nullptr;
}

void ompl::geometric::dRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::dRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::dRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
    
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }
  
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
    
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    
    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());
    
    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
    
    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;
        
        // Now we have random states and nearest states
        ompl::base::ScopedState<> newState(si_);
        for(int i = 0; i < 12; i++){
            newState[i] = 0;
        }

        // Get position coordinate for random state
        const ompl::base::CompoundStateSpace::StateType& rmotionCS = *rstate->as<ompl::base::CompoundStateSpace::StateType>();
        const ompl::base::RealVectorStateSpace::StateType& rmotionPos = *rmotionCS.as<ompl::base::RealVectorStateSpace::StateType>(0);

        // Get position coordinate for nearest state
        const ompl::base::CompoundStateSpace::StateType& nmotionCS = *nmotion->state->as<ompl::base::CompoundStateSpace::StateType>();
        const ompl::base::RealVectorStateSpace::StateType& nmotionPos = *nmotionCS.as<ompl::base::RealVectorStateSpace::StateType>(0);
        
        for(int i = 0; i < 4; i++) {

            unsigned int vertexId = -1;

            for(int j = 0; j < roadMap.size(); j++) {
                const ompl::base::State *roadmapState = roadMap[j];
                const ompl::base::CompoundStateSpace::StateType& roadmapStateCS = *roadmapState->as<ompl::base::CompoundStateSpace::StateType>();
                const ompl::base::RealVectorStateSpace::StateType &roadmapPos = *roadmapStateCS.as<ompl::base::RealVectorStateSpace::StateType>(0);

                // Find the index of nearest state
                if(nmotionPos[i * 2] == roadmapPos[0] && nmotionPos[2 * i + 1] == roadmapPos[1]){
                    //std::cout << "robot: " << i << " with nearest state: " << roadMapStatePos[0] << ", " << roadMapStatePos[1] << " at vertex id: " << j << std::endl; 
                    vertexId = j;
                    break;
                }
            }
        
            // Find connected vertices with nearest state
            // getEdges (unsigned int v, std::vector< unsigned int > &edgeList) const
            std::vector<unsigned int> edgeList;
            roadMapData->getEdges(vertexId, edgeList);
            double max = -1;
            int newStateId = -1;
            double newStateX = 0;
            double newStateY = 0;

            for(int k = 0; k < edgeList.size(); k++){
                // For each connected vertex
                const ompl::base::State *vertex = roadMap[edgeList[k]];
                const ompl::base::CompoundStateSpace::StateType& vertexCS = *vertex->as<ompl::base::CompoundStateSpace::StateType>();	
                const ompl::base::RealVectorStateSpace::StateType &vertexPos = *vertexCS.as<ompl::base::RealVectorStateSpace::StateType>(0);
                const ompl::base::SO2StateSpace::StateType& theta = *vertexCS.as<ompl::base::SO2StateSpace::StateType>(1);

                double randomStateX = rmotionPos[i * 2];
                double randomStateY = rmotionPos[i * 2 + 1];
                //std::cout << "random State position: (" << randomStateX << ": " << randomStateY << ")" << std::endl;

                double nearestStateX = nmotionPos[i * 2];
                double nearestStateY = nmotionPos[i * 2 + 1];
                //std::cout << "nearest State position: (" << nearestStateX << ": " << nearestStateY << ")" << std::endl;

                double roadmapStateX = vertexPos[0];
                double roadmapStateY = vertexPos[1];
                //std::cout << "roadmap State position: (" << roadmapStateX << ": " << roadmapStateY << ")" << std::endl;

                // Vector random state -> nearest state
                double vectorA_X = randomStateX - nearestStateX;
                double vectorA_Y = randomStateY - nearestStateY;

                // Vector random state -> roadmap state
                double vectorB_X = roadmapStateX - nearestStateX;
                double vectorB_Y = roadmapStateY - nearestStateY;

                double cosValue = -1;

                if((vectorA_X != 0 || vectorA_Y != 0) && (vectorB_X != 0 || vectorB_Y != 0)) {
                    // Calculate vectorA and vectorB dot value
                    double vectorADot = vectorA_X * vectorB_X;
                    double vectorBDot = vectorA_Y * vectorB_Y;

                    // Calculate vectorA and vectorB magnitude values
                    double magA = sqrt(vectorA_X * vectorA_X + vectorA_Y * vectorA_Y);
                    double magB = sqrt(vectorB_X * vectorB_X + vectorB_Y * vectorB_Y);

                    cosValue = (vectorADot + vectorBDot) / (magA * magB);
                    //std::cout << "cos value: " << cosValue << std::endl;
                }

                // We keep the largest cosine value as the closest direction of roadmap state to random state
                if(cosValue >= max){
                    max = cosValue;
                    newStateId = k;
                    newStateX = roadmapStateX;
                    newStateY = roadmapStateY;
                    //std::cout << "newState: " << newStateX << ", " << newStateY << std::endl;

                    newState[i * 2] = roadmapStateX;
                    newState[i * 2 + 1] = roadmapStateY;
                    newState[i + 8] = theta.value;
                }
            }  
        }

        bool flag = true;

        std::vector<Rectangle> robotAsRectangle;
        for(int i = 0; i < 4; i++) {
            Rectangle newRec;
            // rectangle position at left corner
            newRec.x = newState[i * 2] - sideLen;
            newRec.y = newState[i * 2 + 1] - sideLen;
            newRec.width = sideLen;
            newRec.height = sideLen;

            robotAsRectangle.push_back(newRec);
        }

        // Then check if four rectangles are collision-free with each other
        for(int j = 0; j < 4; j++) {
            std::vector<Rectangle> obstacleList = robotAsRectangle;
            Rectangle target = obstacleList[j];
            obstacleList.erase(obstacleList.begin() + j);
            
            // Make sure robot x, y is at center
            double x = target.x + sideLen;
            double y = target.y + sideLen;
            double theta = newState[j + 8];
            if(!isValidSquare(x, y, theta, sideLen, obstacleList)) {
                flag = false;
            }
        }

        // If four robots are collision-free then we add them into the Tree
        if(flag == true) {
            auto *newMotion = new Motion(si_);
            si_->copyState(newMotion->state, newState.get());
            newMotion->parent = nmotion;
            nn_->add(newMotion);

            double dist = 0.0;
            bool sat = goal->isSatisfied(newMotion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = newMotion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = newMotion;
            }
        }
    }    
   
    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }
    
    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;
        
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }
        
        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;
    
    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
    
    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::dRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    
    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);
    
    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
    
    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

void ompl::geometric::dRRT::getRoadMap(std::vector<const ompl::base::State*> &roadMap)
{
    this->roadMap = roadMap;
}
