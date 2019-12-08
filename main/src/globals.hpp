#ifndef GLOBAL_HPP
#define GLOBAL_HPP

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>

#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>


// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

// OpenCV 3
#include <opencv2/opencv.hpp>

// Qt
#include <QTimer>


#include <rws/RobWorkStudio.hpp>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <functional>

namespace globals
{
    using namespace rw::common;
    using namespace rw::graphics;
    using namespace rw::kinematics;
    using namespace rw::loaders;
    using namespace rw::models;
    using namespace rw::sensor;
    using namespace rwlibs::opengl;
    using namespace rwlibs::simulation;

    using namespace std;
    using namespace rw::math;
    using namespace rw::pathplanning;
    using namespace rw::proximity;
    using namespace rw::trajectory;
    using namespace rwlibs::pathplanners;
    using namespace rwlibs::proximitystrategies;


    using namespace rws;

    using namespace cv;

    rw::models::WorkCell::Ptr wc;
    rw::kinematics::State state;
    rwlibs::opengl::RenderImage *textureRender, *bgRender;
    rwlibs::simulation::GLFrameGrabber* framegrabber;
    rwlibs::simulation::GLFrameGrabber25D* framegrabber25D;    
    vector<string> cameras;
    vector<string> cameras25D;
    SerialDevice::Ptr device;
    QPath path;
    CollisionDetector::Ptr detector;
    MovableFrame::Ptr dog;
    MovableFrame::Ptr target;
}

#endif /*GLOBAL_HPP*/