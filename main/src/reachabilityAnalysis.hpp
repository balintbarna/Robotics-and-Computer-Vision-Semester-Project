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

struct ReachData
{
	WorkCell::Ptr wc;
	SerialDevice::Ptr robot;
	TreeDevice::Ptr gripper;
	MovableFrame::Ptr targetUp;
	MovableFrame::Ptr targetSide;
	CollisionDetector::Ptr detector;
	MovableFrame::Ptr goal;
	State state;
	vector<State> *states_ptr;
	int num_pos;
};


Transform3D<double> bestRobotPose;
int bestRobotPoseValue = 0;
Q bestPickupConf;
Q bestPlaceConf;

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

int analyse_reachability(ReachData &data)
{
	//load workcell
	if(NULL==data.wc){
		RW_THROW("COULD NOT LOAD scene... check path!");
		return -1;
	}

	// find relevant frames
	if(NULL==data.targetUp){
		RW_THROW("COULD not find movable frame targetUp ... check model");
		return -1;
	}
	if(NULL==data.targetSide){
		RW_THROW("COULD not find movable frame targetSide ... check model");
		return -1;
	}

	if(NULL==data.robot){
		RW_THROW("COULD not find device UR5 ... check model");
		return -1;
	}

	// get the default state
	State &state = data.state;
	auto &states = *data.states_ptr;
	data.gripper->setQ(Q(1, 0.035), state);

	vector<State> bestStates;
	Q pickupConf;
	Q placeConf;

	// set up random seed
	srand(time(NULL));
	// look at 10 random base locations
	auto robRef = globals::robotRef;
	auto startPos = robRef->getTransform(state).P();
	auto startRot = robRef->getTransform(state).R();
	ofstream f;
	f.open("reachData.csv");
    f << "time\tx\ty\tsols" << "\n";
	Timer t;
	for(int i = 0; i < data.num_pos; i++)
	{

		t.resetAndResume();
		cout<<"Trying position "<<i<<endl;
		// move robot to random pos
		double xdif, ydif;
		if(i > 0)
		{
			xdif = (rand() % 1000 - 500) / 1000.0;
			ydif = (rand() % 1000 - 500) / 1000.0;
			robRef->moveTo(Transform3D<>(
				Vector3D<double>(startPos[0] + xdif, startPos[1] + ydif, startPos[2]),
				startRot),
				state);
		}

		// do checks in that pose
		vector<State> collisionFreeStates;
		// picup
		check_target(data.wc,data.robot,data.targetUp,data.targetSide,data.detector,collisionFreeStates, state);
		Q tempPickup;
		Q tempPlace;
		if(collisionFreeStates.size() > 0)
		{
			tempPickup = data.robot->getQ(collisionFreeStates[0]);
		}
		auto statesSize = collisionFreeStates.size();
		if(data.goal != NULL)
		{
			// place
			check_target(data.wc,data.robot,data.goal,data.goal,data.detector,collisionFreeStates, state);
			if(statesSize < collisionFreeStates.size())
			{
				tempPlace = data.robot->getQ(collisionFreeStates[statesSize]);
			}
		}
		if(collisionFreeStates.size() > bestStates.size())
		{
			bestStates = collisionFreeStates;
			pickupConf = tempPickup;
			placeConf = tempPlace;
		}

		t.pause();
		f << t.getTime() << "\t" << xdif << "\t" << ydif << "\t" << collisionFreeStates.size() << "\n";

	}

	int val = bestStates.size();
	if(val > bestRobotPoseValue)
	{
		bestRobotPose = globals::robotRef->getTransform(bestStates[0]);
		bestRobotPoseValue = val;
		bestPickupConf = pickupConf;
		bestPlaceConf = placeConf;
	}

	
	f.close();

	cout << "Current position of the robot vs object to be grasped has: "
		 << bestStates.size()
		 << " collision-free inverse kinematics solutions!" << endl;


	states.insert(std::end(states), std::begin(bestStates), std::end(bestStates));
	auto s = bestStates.size();
	if(s>0)
		states.push_back(bestStates[s-1]);

	return 0;
}

}

#endif /*REACH_HPP*/