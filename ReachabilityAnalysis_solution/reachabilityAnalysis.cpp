
#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>


#include <iostream>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace robwork;


std::vector<rw::math::Q> getConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
    // Get, make and print name of frames
    const std::string robotName = robot->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    rw::kinematics::Frame* frameGoal = wc->findFrame(nameGoal);
    rw::kinematics::Frame* frameTcp = wc->findFrame(nameTcp);
    rw::kinematics::Frame* frameRobotBase = wc->findFrame(nameRobotBase);
    rw::kinematics::Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==NULL || frameTcp==NULL || frameRobotBase==NULL || frameRobotTcp==NULL)
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (frameGoal==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (frameTcp==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==NULL ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    rw::math::Transform3D<> frameBaseTGoal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state);
    rw::math::Transform3D<> frameTcpTRobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state);

    // get grasp frame in robot tool frame
    rw::math::Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(robot, state) );
    return closedFormSovler->solve(targetAt, state);


}



int main(int argc, char** argv)
{
	//load workcell
	rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../scene/Scene.wc.xml");
	if(NULL==wc){
		RW_THROW("COULD NOT LOAD scene... check path!");
		return -1;
	}

	// find relevant frames
	rw::kinematics::MovableFrame::Ptr cylinderFrame = wc->findFrame<rw::kinematics::MovableFrame>("Cylinder");
	if(NULL==cylinderFrame){
		RW_THROW("COULD not find movable frame Cylinder ... check model");
		return -1;
	}

	rw::models::SerialDevice::Ptr robotUR5 = wc->findDevice<rw::models::SerialDevice>("UR5");
	if(NULL==robotUR5){
		RW_THROW("COULD not find device UR5 ... check model");
		return -1;
	}

	rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));


	// get the default state
	State state = wc->getDefaultState();
	std::vector<rw::math::Q> collisionFreeSolutions;

	for(double rollAngle=0; rollAngle<360.0; rollAngle+=1.0){ // for every degree around the roll axis

		cylinderFrame->moveTo(
				rw::math::Transform3D<>(
						rw::math::Vector3D<>(cylinderFrame->getTransform(state).P()),
						rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,0)
						)
				, state);


		std::vector<rw::math::Q> solutions = getConfigurations("GraspTarget", "GraspTCP", robotUR5, wc, state);

		for(unsigned int i=0; i<solutions.size(); i++){
			// set the robot in that configuration and check if it is in collision
			robotUR5->setQ(solutions[i], state);
			if( !detector->inCollision(state,NULL,true) ){
				collisionFreeSolutions.push_back(solutions[i]); // save it
				break; // we only need one
			}
		}
	}

	std:: cout << "Current position of the robot vs object to be grasped has: "
			   << collisionFreeSolutions.size()
			   << " collision-free inverse kinematics solutions!" << std::endl;


	// visualize them
	TimedStatePath tStatePath;
	double time=0;
	for(unsigned int i=0; i<collisionFreeSolutions.size(); i++){
		robotUR5->setQ(collisionFreeSolutions[i], state);
		tStatePath.push_back(TimedState(time,state));
		time+=0.01;
	}

	rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "../scene/visu.rwplay");

	return 0;
}



