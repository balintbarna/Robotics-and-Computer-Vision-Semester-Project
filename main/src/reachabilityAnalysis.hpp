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


vector<Q> getConfigurations(Frame::Ptr frameGoal, const string nameTcp, SerialDevice::Ptr robot, WorkCell::Ptr wc, State state)
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

int analyse_reachability(rws::RobWorkStudio *studio, WorkCell::Ptr wc, SerialDevice::Ptr robot, MovableFrame::Ptr target, CollisionDetector::Ptr detector, bool all)
{
	//load workcell
	if(NULL==wc){
		RW_THROW("COULD NOT LOAD scene... check path!");
		return -1;
	}

	// find relevant frames
	if(NULL==target){
		RW_THROW("COULD not find movable frame Cylinder ... check model");
		return -1;
	}

	if(NULL==robot){
		RW_THROW("COULD not find device UR5 ... check model");
		return -1;
	}

	// get the default state
	State state = wc->getDefaultState();
	vector<Q> collisionFreeSolutions;
	globals::states.clear();

	globals::gripper->setQ(Q(1, 0.045), state);

	for(double rollAngle=0; rollAngle<360.0; rollAngle+=1.0){ // for every degree around the roll axis
		cout<<"Checking angle:"<<rollAngle<<endl;
		target->moveTo(
				Transform3D<>(
						Vector3D<>(target->getTransform(state).P()),
						// RPY<>(rollAngle*Deg2Rad,0,0) // original
						// RPY<>(0,rollAngle*Deg2Rad,0) // from side
						RPY<>(0,rollAngle*Deg2Rad,90*Deg2Rad) // from upwards
						)
				, state);

		vector<Q> solutions = getConfigurations(target, "GraspTCP", robot, wc, state);
		for(auto &sol : solutions)
		{
			if(all)
				globals::states.push_back(state);

			robot->setQ(sol, state);
			// set the robot in that configuration and check if it is in collision
			if( !detector->inCollision(state,NULL,true) ){
				if(all == false)
					globals::states.push_back(state);

				collisionFreeSolutions.push_back(sol); // save it
				break; // we only need one
			}
		}
	}

	 cout << "Current position of the robot vs object to be grasped has: "
			   << collisionFreeSolutions.size()
			   << " collision-free inverse kinematics solutions!" << endl;


	return 0;
}

int analyse_reachability(rws::RobWorkStudio *studio, WorkCell::Ptr wc, SerialDevice::Ptr robot, MovableFrame::Ptr target, CollisionDetector::Ptr detector)
{
	analyse_reachability(studio, wc, robot, target, detector, false);
}

#endif /*REACH_HPP*/