#ifndef REACH_HPP
#define REACH_HPP

#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>


#include <iostream>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::proximity;
using namespace rw::common;
using namespace rwlibs::proximitystrategies;
using namespace rw::invkin;


vector<Q> getConfigurations(Frame::Ptr frameGoal, const string &nameTcp, SerialDevice::Ptr robot, WorkCell::Ptr wc, State state)
{
    // Get, make and print name of frames
    const string robotName = robot->getName();
    const string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    Frame* frameTcp = wc->findFrame(nameTcp);
    Frame* frameRobotBase = robot->getBase();
    Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==NULL || frameTcp==NULL || frameRobotBase==NULL || frameRobotTcp==NULL)
    {
        cout << " ALL FRAMES NOT FOUND:" << endl;
        cout << " Found \"" << "goal" << "\": " << (frameGoal==NULL ? "NO!" : "YES!") << endl;
        cout << " Found \"" << nameTcp << "\": " << (frameTcp==NULL ? "NO!" : "YES!") << endl;
        cout << " Found \"" << "robot base" << "\": " << (frameRobotBase==NULL ? "NO!" : "YES!") << endl;
        cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==NULL ? "NO!" : "YES!") << endl;
    }

    // Make "helper" transformations
    Transform3D<> frameBaseTGoal = Kinematics::frameTframe(frameRobotBase, &(*frameGoal), state);
    Transform3D<> frameTcpTRobotTcp = Kinematics::frameTframe(frameTcp, frameRobotTcp, state);

    // get grasp frame in robot tool frame
    Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

    ClosedFormIKSolverUR::Ptr closedFormSovler = ownedPtr( new ClosedFormIKSolverUR(robot, state) );
    return closedFormSovler->solve(targetAt, state);
}

void check_solutions(vector<Q> solutions, SerialDevice::Ptr robot, CollisionDetector::Ptr detector, vector<State> &states, State &state, bool all)
{
	for(auto &sol : solutions)
	{
		if(all)
			globals::states.push_back(state);

		robot->setQ(sol, state);
		// set the robot in that configuration and check if it is in collision
		if( !detector->inCollision(state,NULL,true) ){
			if(all == false)
				globals::states.push_back(state);

			states.push_back(state); // save it
			break; // we only need one
		}
	}
}

void check_target(WorkCell::Ptr wc, SerialDevice::Ptr robot, MovableFrame::Ptr targetUp, MovableFrame::Ptr targetSide, CollisionDetector::Ptr detector, vector<State> &states, State &state, bool all)
{
	auto tpos = targetUp->getTransform(state).P();
	// for every degree around the roll axis
	for(double angle=0; angle<360.0; angle+=1.0)
	{
		cout<<"Checking angle:"<<angle<<endl;
		targetUp->moveTo(
				Transform3D<>(
						tpos,
						// RPY<>(angle*Deg2Rad,0,0) // original
						// RPY<>(0,angle*Deg2Rad,0) // from side
						RPY<>(0,angle*Deg2Rad,90*Deg2Rad) // from upwards
						)
				, state);

		vector<Q> solutions = getConfigurations(targetUp, "GraspTCP", robot, wc, state);
		check_solutions(solutions, robot, detector, states, state, all);
	}
	tpos = targetSide->getTransform(state).P();
	for(double angle=0; angle<360.0; angle+=1.0)
	{
		cout<<"Checking angle:"<<angle<<endl;
		targetSide->moveTo(
				Transform3D<>(
						tpos,
						// RPY<>(angle*Deg2Rad,0,0) // original
						RPY<>(0,angle*Deg2Rad,0) // from side
						// RPY<>(0,angle*Deg2Rad,90*Deg2Rad) // from upwards
						)
				, state);

		vector<Q> solutions = getConfigurations(targetSide, "GraspTCP", robot, wc, state);
		check_solutions(solutions, robot, detector, states, state, all);
	}
}

int analyse_reachability(WorkCell::Ptr wc, SerialDevice::Ptr robot, MovableFrame::Ptr targetUp, MovableFrame::Ptr targetSide, CollisionDetector::Ptr detector, bool all, MovableFrame::Ptr goal)
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

	globals::states.clear();
	vector<State> collisionFreeStates;

	check_target(wc,robot,targetUp,targetSide,detector,collisionFreeStates, state, all);
	if(goal != NULL)
	{
		check_target(wc,robot,goal,goal,detector,collisionFreeStates, state, all);
	}
	

	cout << "Current position of the robot vs object to be grasped has: "
		 << collisionFreeStates.size()
		 << " collision-free inverse kinematics solutions!" << endl;


	return 0;
}

#endif /*REACH_HPP*/