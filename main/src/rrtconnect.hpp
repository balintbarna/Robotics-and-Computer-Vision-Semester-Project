#ifndef RRTCONNECT_HPP
#define RRTCONNECT_HPP

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 60.

namespace rrtconnect
{

Transform3D<double> relativeTransformCalc(Frame::Ptr targetFrame, State &state)
{
    // Make "helper" transformations
    auto frameBaseTGoal = Kinematics::frameTframe(globals::robot->getBase(), targetFrame.get(), state);
    auto frameTcpTRobotTcp = Kinematics::frameTframe(globals::graspTcp.get(), globals::robotTcp.get(), state);
    // get grasp frame in robot tool frame
    auto relative = frameBaseTGoal * frameTcpTRobotTcp;
    return relative;
}

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState = state;
	CollisionDetector::QueryResult data;
	bool colFrom;

	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
    return true;
}

void createPath(Q from, Q to,  double extend, double maxTime)
{
    State state = globals::state;
    auto robot = globals::robot;
    robot->setQ(from,state);
    CollisionDetector detector(globals::wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,robot,state);
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(robot),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    QPath path;
    if (!checkCollisions(robot, state, detector, from))
        cout << from << " is in colission!" << endl;
    if (!checkCollisions(robot, state, detector, to))
        cout << to << " is in colission!" << endl;;
    Timer t;
    t.resetAndResume();
    planner->query(from,to,path,maxTime);
    t.pause();


    if (t.getTime() >= maxTime) {
        cout << "Notice: max time of " << maxTime << " seconds reached." << endl;
    }

	const int duration = 10;

    if(path.size() == 2)
	{  //The interpolated path between Q start and Q goal is collision free. Set the duration with respect to the desired velocity
        LinearInterpolator<Q> linInt(from, to, duration);
        QPath tempQ;
        for(int i = 0; i < duration+1; i++){
            tempQ.push_back(linInt.x(i));
        }

        path=tempQ;
    }
    globals::states.clear();
    for(auto &q : path)
    {
        robot->setQ(q, state);
        globals::states.push_back(state);
    }
}

int main_backup(int argc, char** argv) {

    ofstream mydata;
    mydata.open("ROBDATA.dat");
    mydata << "time\tdistance\teps\tsteps" << "\n";
    mydata.close();

    mydata.open("ROBDATA.dat", std::ios_base::app);

    for (double extend = 0.02; extend <= 1.0; extend+=0.02)
    {
        for(int trial = 0; trial < 40; trial++)
        {
            const string wcFile = "./Kr16WallWorkCell/Scene.wc.xml";
            const string deviceName = "KukaKr16";
            //cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;
            ofstream myfile;
            myfile.open("path.lua");
            rw::math::Math::seed();

            WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
            Frame *tool_frame = wc->findFrame("Tool");
            Frame *bottle_frame = wc->findFrame("Bottle");

            Device::Ptr device = wc->findDevice(deviceName);        //process finished with exit code 139 (interrupted by signal 11: SIGSEGV)
            if (device == NULL) {
                cerr << "Device: " << deviceName << " not found!" << endl;
                return 0;
            }

            State state = wc->getDefaultState();
            Q from(6,-3.142, -0.827, -3.002, -3.143, 0.099, -1.573);
            Q to(6,1.571, 0.006, 0.030, 0.153, 0.762, 4.490);
            device->setQ(from,state);
            Kinematics::gripFrame(bottle_frame, tool_frame, state);
            CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
            PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

            QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
            QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
            QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

            if (!checkCollisions(device, state, detector, from))
                return 0;
            if (!checkCollisions(device, state, detector, to))
                return 0;

            myfile << "wc = rws.getRobWorkStudio():getWorkCell()\n"
                      <<"state = wc:getDefaultState()"
                      <<"\ndevice = wc:findDevice(\"KukaKr16\")"
                      <<"\ngripper = wc:findFrame(\"Tool\")"
                      <<"\nbottle = wc:findFrame(\"Bottle\")\n"
                      <<"table = wc:findFrame(\"Table\")\n\n"
                      <<"function setQ(q)\n"
                      <<"qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n"
                      <<"device:setQ(qq,state)\n"
                      <<"rws.getRobWorkStudio():setState(state)\n"
                      <<"rw.sleep(0.1)\n"
                      <<"end\n\n"
                      <<"function attach(obj, tool)\n"
                      <<"rw.gripFrame(obj, tool, state)\n"
                      <<"rws.getRobWorkStudio():setState(state)\n"
                      <<"rw.sleep(0.1)\n"
                      <<"end\n\n";

            //cout << "Planning from " << from << " to " << to << endl;
            QPath path;
            Timer t;
            t.resetAndResume();
            planner->query(from,to,path,MAXTIME);
            t.pause();
            double distance = 0;


            //cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;
            if (t.getTime() >= MAXTIME) {
                cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
            }

            for (unsigned int i = 0; i< path.size(); i++)
            {
                if(i == 1)
                    myfile << "attach(bottle, gripper)\n";
                if(i >= 1)
                    distance += sqrt(pow((path.at(i)(0)-path.at(i-1)(0)),2)+pow((path.at(i)(1)-path.at(i-1)(1)),2)+pow((path.at(i)(2)-path.at(i-1)(2)),2)+pow((path.at(i)(3)-path.at(i-1)(3)),2)+pow((path.at(i)(4)-path.at(i-1)(4)),2)+pow((path.at(i)(5)-path.at(i-1)(5)),2));
                //cout << path.at(i)(0) << endl;
                myfile <<"setQ({" << path.at(i)(0) << "," << path.at(i)(1) << "," << path.at(i)(2) << "," << path.at(i)(3) << "," << path.at(i)(4) << "," << path.at(i)(5) << "})" << "\n";
            }
            mydata << t.getTime() << "\t" << distance << "\t" << extend << "\t" << path.size() << "\n";

            myfile.close();
            cout << trial << endl;
        }
    }

    mydata.close();
	return 0;
}

}

#endif /*RRTCONNECT_HPP*/