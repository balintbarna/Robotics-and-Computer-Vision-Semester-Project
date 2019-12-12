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

namespace rrtconnect
{
    struct RrtData
    {
        WorkCell::Ptr wc;
        SerialDevice::Ptr robot;
        Q from;
        Q to;
        double extend_start;
        double extend_step;
        double extend_max;

        State state;
        vector<State> *states;
    };

    double bestDistance = 99999999;
    QPath bestPath;

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

QPath createPath(Q from, Q to,  double extend, double maxTime, State state)
{
    auto robot = globals::robot;
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
    return path;
}

void test_rrt(RrtData data) {

    ofstream mydata;
    mydata.open("rrtPerfData.dat");
    mydata << "time\tdistance\teps\tsteps" << "\n";
    mydata.close();

    mydata.open("rrtPerfData.dat", std::ios_base::app);

    double MAXTIME = 60;

    State &startState = data.state;

    for (double extend = data.extend_start; extend <= data.extend_max; extend+=data.extend_step)
    {
        cout<<"extend:"<<extend<<endl;
        for(int trial = 0; trial < 10; trial++)
        {
            Timer t;
            t.resetAndResume();
            QPath path = createPath(data.from, data.to, extend, MAXTIME, startState);
            t.pause();
            double distance = 0;

            // STARTS FROM SECOND POSE
            for (size_t i = 1; i < path.size(); i++)
            {
                distance += sqrt(pow((path.at(i)(0)-path.at(i-1)(0)),2)+pow((path.at(i)(1)-path.at(i-1)(1)),2)+pow((path.at(i)(2)-path.at(i-1)(2)),2)+pow((path.at(i)(3)-path.at(i-1)(3)),2)+pow((path.at(i)(4)-path.at(i-1)(4)),2)+pow((path.at(i)(5)-path.at(i-1)(5)),2));

            }
            mydata << t.getTime() << "\t" << distance << "\t" << extend << "\t" << path.size() << "\n";

            cout << "trial:" << trial << endl;
            if(distance < bestDistance)
            {
                bestDistance = distance;
                bestPath = path;
                cout<<"found new best path, distance:"<<distance<<endl;
            }
        }
    }

    mydata.close();

    ofstream f;
	f.open("bestRrtPath.dat");
    f << "x\ty\ty" << endl;
    for(auto &q : bestPath)
    {
        data.robot->setQ(q, startState);
        auto baseTdog = data.robot->baseTframe(globals::dog.get(), startState);
        f<<baseTdog.P()[0]<<"\t"<<baseTdog.P()[1]<<"\t"<<baseTdog.P()[2]<<endl;
        data.states->push_back(startState);
    }
    f.close();
    cout<<"saved best path"<<endl;
}

}

#endif /*RRTCONNECT_HPP*/