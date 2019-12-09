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
    using namespace std;
    using namespace cv;

    using namespace rw::common;
    using namespace rw::graphics;
    using namespace rw::kinematics;
    using namespace rw::loaders;
    using namespace rw::models;
    using namespace rw::sensor;
    using namespace rw::math;
    using namespace rw::pathplanning;
    using namespace rw::proximity;
    using namespace rw::trajectory;
    using namespace rwlibs::opengl;
    using namespace rwlibs::simulation;
    using namespace rwlibs::pathplanners;
    using namespace rwlibs::proximitystrategies;
    using namespace rws;

    WorkCell::Ptr wc;
    State state;
    RenderImage *textureRender, *bgRender;
    GLFrameGrabber* framegrabber;
    GLFrameGrabber25D* framegrabber25D;    
    vector<string> cameras;
    vector<string> cameras25D;
    MovableFrame::Ptr robotRef;
    SerialDevice::Ptr robot;
    TreeDevice::Ptr gripper;
    CollisionDetector::Ptr detector;
    MovableFrame::Ptr target;
    Frame::Ptr graspTcp;
    Frame::Ptr robotTcp;
    MovableFrame::Ptr dog;
    MovableFrame::Ptr dogmiddle;
    MovableFrame::Ptr doghead;
    MovableFrame::Ptr goal;
    vector<State> states;

    void init(WorkCell *workcell, RobWorkStudioPlugin *plugin)
    {
        wc = workcell;
        state = wc->getDefaultState();
        auto studio = plugin->getRobWorkStudio();

        if (wc != NULL)
        {
            // Add the texture render to this workcell if there is a frame for texture
            Frame* textureFrame = wc->findFrame("MarkerTexture");
            if (textureFrame != NULL)
            {
                studio->getWorkCellScene()->addRender("TextureImage",textureRender,textureFrame);
            }
            // Add the background render to this workcell if there is a frame for texture
            Frame* bgFrame = wc->findFrame("Background");
            if (bgFrame != NULL)
            {
                studio->getWorkCellScene()->addRender("BackgroundImage",bgRender,bgFrame);
            }

            // Create a GLFrameGrabber if there is a camera frame with a Camera property set
            Frame* cameraFrame = wc->findFrame(cameras[0]);
            if (cameraFrame != NULL)
            {
                if (cameraFrame->getPropertyMap().has("Camera"))
                {
                    // Read the dimensions and field of view
                    double fovy;
                    int width,height;
                    std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
                    std::istringstream iss (camParam, std::istringstream::in);
                    iss >> fovy >> width >> height;
                    // Create a frame grabber
                    framegrabber = new GLFrameGrabber(width,height,fovy);
                    SceneViewer::Ptr gldrawer = studio->getView()->getSceneViewer();
                    framegrabber->init(gldrawer);
                }
            }
            
            Frame* cameraFrame25D = wc->findFrame(cameras25D[0]);
            if (cameraFrame25D != NULL)
            {
                if (cameraFrame25D->getPropertyMap().has("Scanner25D"))
                {
                    // Read the dimensions and field of view
                    double fovy;
                    int width,height;
                    std::string camParam = cameraFrame25D->getPropertyMap().get<std::string>("Scanner25D");
                    std::istringstream iss (camParam, std::istringstream::in);
                    iss >> fovy >> width >> height;
                    // Create a frame grabber
                    framegrabber25D = new GLFrameGrabber25D(width,height,fovy);
                    SceneViewer::Ptr gldrawer = studio->getView()->getSceneViewer();
                    framegrabber25D->init(gldrawer);
                }
            }

            robotRef = wc->findFrame<MovableFrame>("URReference");
            robot = wc->findDevice<SerialDevice>("UR-6-85-5-A");
            gripper = wc->findDevice<TreeDevice>("WSG50");
            detector = ownedPtr(new CollisionDetector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy()));
            target = wc->findFrame<MovableFrame>("GraspTarget");
            graspTcp = wc->findFrame("GraspTCP");
            dog = wc->findFrame<MovableFrame>("Dog");
            dogmiddle = wc->findFrame<MovableFrame>("DogMiddle");
            doghead = wc->findFrame<MovableFrame>("DogHead");
            goal = wc->findFrame<MovableFrame>("Goal");
            robotTcp = wc->findFrame(robot->getName() + "." + "TCP");
        }
    }

    void close(RobWorkStudioPlugin *plugin)
    {
        auto studio = plugin->getRobWorkStudio();
        Frame* textureFrame = wc->findFrame("MarkerTexture");
        if (textureFrame != NULL)
        {
            studio->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
        }
        // Remove the background render
        Frame* bgFrame = wc->findFrame("Background");
        if (bgFrame != NULL)
        {
            studio->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
        }
        // Delete the old framegrabber
        if (framegrabber != NULL)
        {
            delete framegrabber;
        }
        framegrabber = NULL;
        if (framegrabber25D != NULL)
        {
            delete framegrabber25D;
        }
        framegrabber25D = NULL;
        wc = NULL;
    }
}

#endif /*GLOBALS*/