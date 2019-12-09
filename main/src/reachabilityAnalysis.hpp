#ifndef REACH_HPP
#define REACH_HPP

#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>


#include <iostream>
#include <string>
#include <random>
#include <time.h> 

namespace reach
{

using namespace std;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::proximity;
using namespace rw::common;
using namespace rwlibs::proximitystrategies;
using namespace rw::invkin;

vector<Q> getConfigurations(Frame::Ptr frameGoal, Frame::Ptr frameTcp, SerialDevice::Ptr robot, WorkCell::Ptr wc, State &state)
{
    // Get, make and print name of frames
    const string robotName = robot->getName();
    const string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    Frame* frameRobotBase = robot->getBase();
    Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==NULL || frameTcp==NULL || frameRobotBase==NULL || frameRobotTcp==NULL)
    {
        cout << " ALL FRAMES NOT FOUND:" << endl;
        cout << " Found \"" << "goal" << "\": " << (frameGoal==NULL ? "NO!" : "YES!") << endl;
        cout << " Found \"" << "gripper tcp" << "\": " << (frameTcp==NULL ? "NO!" : "YES!") << endl;
        cout << " Found \"" << "robot base" << "\": " << (frameRobotBase==NULL ? "NO!" : "YES!") << endl;
        cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==NULL ? "NO!" : "YES!") << endl;
    }

    // Make "helper" transformations
    Transform3D<> frameBaseTGoal = Kinematics::frameTframe(frameRobotBase, frameGoal.get(), state);
    Transform3D<> frameTcpTRobotTcp = Kinematics::frameTframe(frameTcp.get(), frameRobotTcp, state);

    // get grasp frame in robot tool frame
    Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

    ClosedFormIKSolverUR::Ptr closedFormSovler = ownedPtr( new ClosedFormIKSolverUR(robot, state) );
    return closedFormSovler->solve(targetAt, state);
}

void check_solutions(vector<Q> &solutions, SerialDevice::Ptr robot, CollisionDetector::Ptr detector, vector<State> &states, State &state)
{
	for(auto &sol : solutions)
	{
		robot->setQ(sol, state);
		// set the robot in that configuration and check if it is in collision
		if(!detector->inCollision(state,NULL,true)){
			states.push_back(state); // save it
			break; // we only need one
		}
	}
}

void check_target(WorkCell::Ptr wc, SerialDevice::Ptr robot, MovableFrame::Ptr targetUp, MovableFrame::Ptr targetSide, CollisionDetector::Ptr detector, vector<State> &states, State &state)
{
	globals::target->attachTo(targetUp.get(), state);
	Vector3D<double> zeroPos(0, 0, 0);
	// for every degree around the roll axis
	for(double angle=0; angle<360.0; angle+=1.0)
	{
		globals::target->moveTo(
				Transform3D<>(
						zeroPos,
						// RPY<>(angle*Deg2Rad,0,0) // original
						// RPY<>(0,angle*Deg2Rad,0) // from side
						RPY<>(0,angle*Deg2Rad,90*Deg2Rad) // from upwards
						)
				, state);

		vector<Q> solutions = getConfigurations(globals::target, globals::graspTcp, robot, wc, state);
		check_solutions(solutions, robot, detector, states, state);
	}
	globals::target->attachTo(targetSide.get(), state);
	for(double angle=0; angle<360.0; angle+=1.0)
	{
		globals::target->moveTo(
				Transform3D<>(
						zeroPos,
						// RPY<>(angle*Deg2Rad,0,0) // original
						RPY<>(0,angle*Deg2Rad,0) // from side
						// RPY<>(0,angle*Deg2Rad,90*Deg2Rad) // from upwards
						)
				, state);

		vector<Q> solutions = getConfigurations(globals::target, globals::graspTcp, robot, wc, state);
		check_solutions(solutions, robot, detector, states, state);
	}
}

int analyse_reachability(WorkCell::Ptr wc, SerialDevice::Ptr robot, MovableFrame::Ptr targetUp, MovableFrame::Ptr targetSide, CollisionDetector::Ptr detector, MovableFrame::Ptr goal, int num_pos)
{
	//load workcell
	if(NULL==wc){
		RW_THROW("COULD NOT LOAD scene... check path!");
		return -1;
	}

	// find relevant frames
	if(NULL==targetUp){
		RW_THROW("COULD not find movable frame targetUp ... check model");
		return -1;
	}
	if(NULL==targetSide){
		RW_THROW("COULD not find movable frame targetSide ... check model");
		return -1;
	}

	if(NULL==robot){
		RW_THROW("COULD not find device UR5 ... check model");
		return -1;
	}

	// get the default state
	State state = wc->getDefaultState();
	globals::gripper->setQ(Q(1, 0.045), state);

	vector<State> bestStates;

	// set up random seed
	srand(time(NULL));
	// look at 10 random base locations
	auto robRef = globals::robotRef;
	auto startPos = robRef->getTransform(state).P();
	auto startRot = robRef->getTransform(state).R();
	for(int i = 0; i < num_pos; i++)
	{
		// move robot to random pos
		if(i > 0)
		{
		double xdif = (rand() % 1000 - 500) / 1000.0;
		double ydif = (rand() % 1000 - 500) / 1000.0;
		robRef->moveTo(Transform3D<>(
			Vector3D<double>(xdif, ydif, 0),
			startRot),
			state);
		}

		// do checks in that pose
		vector<State> collisionFreeStates;
		check_target(wc,robot,targetUp,targetSide,detector,collisionFreeStates, state);
		if(goal != NULL)
		{
			check_target(wc,robot,goal,goal,detector,collisionFreeStates, state);
		}
		if(collisionFreeStates.size() > bestStates.size())
		{
			bestStates = collisionFreeStates;
		}
	}

	globals::states = bestStates;

	int val = bestStates.size();
	if(val > globals::bestRobotPoseValue)
	{
		globals::bestRobotPose = globals::robotRef->getTransform(bestStates[0]);
		globals::bestRobotPoseValue = val;
	}
	

	cout << "Current position of the robot vs object to be grasped has: "
		 << bestStates.size()
		 << " collision-free inverse kinematics solutions!" << endl;


	return 0;
}

}

#endif /*REACH_HPP*/