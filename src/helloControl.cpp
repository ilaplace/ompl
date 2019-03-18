//#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
//#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
//#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/util/PPM.h>
#include <boost/filesystem.hpp>
//did i just fucked
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

class Recorder
{
    public:
    void recordSolution(oc::SimpleSetup ss_, ompl::PPM &ppm_)
    {
    
        oc::PathControl &p = ss_.getSolutionPath();
        p.interpolate();
        
       
        //std::cout << "State for loop "<< std::endl;
        for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
        {
            //TODo Set width height
            const int w = std::min(1800, (int)p.getState(i)->as<ob::SE2StateSpace::StateType>()->getX());
            const int h = std::min(1800, (int)p.getState(i)->as<ob::SE2StateSpace::StateType>()->getY());
            ompl::PPM::Color &c = ppm_.getPixel(h, w);
           // std::cout <<"State number "<< i<<": " << h << " "<< w << std::endl;
            c.red = 255;
            c.green = 0;
            c.blue = 0;
        }
    }
    void save(const char *filename,ompl::PPM &ppm_)
    {
        ppm_.saveFile(filename);
    }
};
void loadWorld(ompl::PPM &ppm_, int &maxWidth_, int &maxHeight_)
{
    boost::filesystem::path path("../resources");
    bool ok = false;
    try
    {
        ppm_.loadFile((path / "floor.ppm").string().c_str());
        ok = true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    if (ok)
    {
        maxWidth_ = ppm_.getWidth();
        maxHeight_ = ppm_.getHeight();

        std::cout << "Creating a map: " << maxWidth_ << " by " << maxWidth_ << std::endl;
    }
}

void SimpleCarODE(const oc::ODESolver::StateType &q, const oc::Control *c, oc::ODESolver::StateType &qdot)
{
    // Retrieve control values.  Velocity is the first entry, steering angle is second.
    const double *u = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double velocity = u[0];
    const double steeringAngle = u[1];
    // Retrieve the current orientation of the car.  The memory for ompl::base::SE2StateSpace is mapped as:
    // 0: x
    // 1: y
    // 2: theta
    const double theta = q[2];
    // Ensure qdot is the same size as q.  Zero out all values.
    qdot.resize(q.size(), 0);
    qdot[0] = velocity * cos(theta);         // x-dot
    qdot[1] = velocity * sin(theta);         // y-dot
    qdot[2] = velocity * tan(steeringAngle); // theta-dot
}
void postPropagate(const ob::State *state, const oc::Control *control, const double duration, ob::State *result)
{
    ompl::base::SO2StateSpace SO2;
    // Ensure that the car's resulting orientation lies between 0 and 2*pi.
    ompl::base::SE2StateSpace::StateType &s = *result->as<ompl::base::SE2StateSpace::StateType>();
    SO2.enforceBounds(s[1]);
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state, ompl::PPM &ppm_)
{   
    //ob::ScopedState<ob::SE2StateSpace>
    //cast the abstract state type to the type we expect
    //const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    //const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
    //const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);
    //To-Do: With and height must be reset
    //onst int w = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], 1000);
    //const int h = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], 1000);
    const int w = std::min((int)state->as<ob::SE2StateSpace::StateType>()->getX(), 1800);
    const int h = std::min((int)state->as<ob::SE2StateSpace::StateType>()->getY(), 1800);

    const ompl::PPM::Color &c = ppm_.getPixel(h, w);
    //std::cout <<h<< " " << w << std::endl;
    //return c.red > 127 && c.green > 127 && c.blue > 127 && si->satisfiesBounds(state) && (const void *)rot != (const void *)pos;
    return c.red > 127 && c.green > 127 && c.blue > 127 && si->satisfiesBounds(state);
}

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
    const double *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
    const double *ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    result->as<ob::SE2StateSpace::StateType>()->setXY(
        pos[0] + ctrl[0] * duration * cos(rot),
        pos[1] + ctrl[0] * duration * sin(rot));
    result->as<ob::SE2StateSpace::StateType>()->setYaw(
        rot + ctrl[1] * duration);
}

void plan(int maxWidth_, int maxHeight_, ompl::PPM &ppm_)
{
    auto space(std::make_shared<ob::SE2StateSpace>());
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(1800);

    space->setBounds(bounds);
    std::cout << "space set set";

    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0);
    cbounds.setHigh(0.3);

    cspace->setBounds(cbounds);

    oc::SimpleSetup ss(cspace);
    std::cout << "Will set the prop" << std::endl;
    ss.setStatePropagator(propagate);
    

    //ToDo: Resolution is not set
    auto si = ss.getSpaceInformation();
    std::cout << "Validty checker is set" << std::endl;
    si->setStateValidityCheckingResolution(0.002);

    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(50.5);
    start->setY(50.0);
    start->setYaw(0.0);

    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(800.0);
    goal->setY(1000.5);
    goal->setYaw(0.5);

    ss.setStartAndGoalStates(start, goal, 0.05);
    std::cout << "Goal and start states are set" << std::endl;

    auto planner(std::make_shared<oc::RRT>(ss.getSpaceInformation()));
    ss.setPlanner(planner);
    ss.setStateValidityChecker([&ss, &ppm_](const ob::State *state) {
        return isStateValid(ss.getSpaceInformation().get(), state, ppm_);
    });

    ob::PlannerStatus solved = ss.solve(300.0);

    if (solved)
    {
        ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
        Recorder record;
        record.recordSolution(ss, ppm_) ;
        record.save("result.ppm",ppm_);
        
    }
    else
    {
        std::cout << "no solution found" << std::endl;
    }
}


int main(int /*argc*/, char ** /*argv*/)
{
    ompl::PPM ppm_;
    int maxWidth_;
    int maxHeight_;

    
    loadWorld(ppm_, maxWidth_, maxHeight_);

    plan(maxWidth_, maxHeight_, ppm_);
    return 0;
}