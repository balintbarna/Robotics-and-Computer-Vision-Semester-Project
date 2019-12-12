#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/random.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include "SamplePlugin.hpp"
#include "globals.hpp"
#include "reachabilityAnalysis.hpp"
#include "rrtconnect.hpp"
#include "imager.hpp"
#include "pcl_functions.hpp"


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

using namespace std::placeholders;

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// slider
	connect(_slider, SIGNAL(sliderMoved(int)), this, SLOT(onSliderMoved()));
	connect(_slider, SIGNAL(sliderPressed()), this, SLOT(onSliderPressed()));
	connect(_slider, SIGNAL(sliderReleased()), this, SLOT(onSliderReleased()));

	// nowbuttons
	connect(_btn_integrated, SIGNAL(pressed()), this, SLOT(btnPressed()));
	connect(_btn_detect, SIGNAL(pressed()), this, SLOT(btnPressed()));
	connect(_btn_reach, SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_state_playback, SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_reset_playback, SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_im, SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_scan, SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_calcpath, SIGNAL(pressed()), this, SLOT(btnPressed()) );

	//spinner
	connect(_spinBox, SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

	globals::framegrabber = NULL;
	
	globals::cameras = {"Camera_Right", "Camera_Left"};
	globals::cameras25D = {"Scanner25D"};
}

SamplePlugin::~SamplePlugin()
{
    delete globals::textureRender;
    delete globals::bgRender;
}

void SamplePlugin::initialize() {
	log().info() << "INITALIZE" << "\n";

    getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, std::placeholders::_1), this);

	// Auto load workcell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/student/Workspace/RoVi_project/Workcell/Scene.wc.xml");
	getRobWorkStudio()->setWorkCell(wc);
}

void SamplePlugin::open(WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
	globals::init(workcell, this);

    log().info() << workcell->getFilename() << "\n";

    _step = 0;
}


void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
	globals::close(this);
}

bool playbackWasOn = false;
void SamplePlugin::onSliderMoved()
{
	// cout<<"slider moved"<<endl;
	auto s = globals::states.size();
    if(s > 0)
	{
		_slider->setMaximum(s - 1);
    }
	_step = _slider->value();
	updatePlaybackState();
}
void SamplePlugin::onSliderPressed()
{
	// cout<<"slider pressed"<<endl;
	playbackWasOn = _timer->isActive();
	_timer->stop();
	auto s = globals::states.size();
    if(s > 0)
	{
		_slider->setMaximum(s - 1);
    }
}
void SamplePlugin::onSliderReleased()
{
	// cout<<"slider released"<<endl;
	auto s = globals::states.size();
    if(s > 0)
	{
		_slider->setMaximum(s - 1);
    }
	_step = _slider->value();
	updatePlaybackState();
	if (playbackWasOn)
		_timer->start();
}

void SamplePlugin::detect_dog()
{
	using namespace pointcloud;
	auto scene = capture_pointcloud();
	auto object = load_object();
	auto detected_pose = pointcloud::do_3d_alignment(scene, object);
	State state = globals::getState();
	globals::detected->moveTo(detected_pose, state);
	globals::dogmiddle->attachTo(globals::detected.get(), state);
	globals::doghead->attachTo(globals::detected.get(), state);
	globals::states.push_back(state);
	globals::states.push_back(state);
}
void SamplePlugin::analyze_reach()
{
	int rounds = _spinBox->value();
	rounds = rounds == 0 ? 1 : rounds;
	reach::ReachData rdata;
	rdata.wc = globals::wc;
	rdata.robot = globals::robot;
	rdata.gripper = globals::gripper;
	rdata.targetUp = globals::dogmiddle;
	rdata.targetSide = globals::doghead;
	rdata.detector = globals::detector;
	rdata.goal = globals::goal;
	rdata.num_pos = rounds;
	rdata.state = globals::getState();
	rdata.states_ptr = &globals::states;

	reach::analyse_reachability(rdata);
}
void SamplePlugin::plan_path()
{
	cout<<"preparing for calculating path"<<endl;
	// pause playback
	_timer->stop();

	rrtconnect::RrtData data;
	data.robot = globals::robot;
	data.wc = globals::wc;
	data.from = reach::bestPickupConf;
	data.to = reach::bestPlaceConf;
	data.extend_start = 0.02;
	data.extend_max = 0.2;
	data.extend_step = data.extend_start;
	data.states = &globals::states;
	data.state = globals::getState();
	
	// set up state
	globals::gripper->setQ(Q(1, 0.045), data.state);
	globals::robotRef->moveTo(reach::bestRobotPose, data.state);
	globals::robot->setQ(data.from, data.state);
	// rw::invkin::ClosedFormIKSolverUR::Ptr solver = ownedPtr(new rw::invkin::ClosedFormIKSolverUR(globals::robot, globals::state));
	// auto toTarget = rrtconnect::relativeTransformCalc(globals::goal, globals::state);
	// Q to = solver->solve(toTarget, globals::state)[0];
	Kinematics::gripFrame(globals::dog.get(), globals::graspTcp.get(), data.state);
	setCurrentState();

	rw::math::Math::seed();
	cout<<"calculating path"<<endl;
	rrtconnect::test_rrt(data);

	cout<<"path calculated"<<endl;
}

void SamplePlugin::btnPressed() {
    QObject *obj = sender();
	if(obj == _btn_state_playback)
	{
		if (!_timer->isActive())
		{
			cout<<"starting playback"<<endl;
            _timer->start(25); // ms
        }
        else
		{
			cout<<"stopping playback"<<endl;
			_timer->stop();
		}
	}
	else if(obj == _btn_reset_playback)
	{
		cout<<"resetting playback"<<endl;
		_timer->stop();
		_step = 0;
		_slider->setValue(_step);
		updatePlaybackState();
	}
	else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
	else if(obj == _btn_detect)
	{
		cout<<"detect"<<endl;
		setDefaultState();
		detect_dog();
		setCurrentState();
	}
	else if(obj == _btn_reach)
	{
		cout<<"reach"<<endl;
		setDefaultState();
		analyze_reach();
		setCurrentState();
	}
	else if(obj==_btn_calcpath){
		setDefaultState();
		cout<<"plan"<<endl;
		plan_path();
		setCurrentState();
	}
	else if(obj == _btn_integrated)
	{
		cout<<"integrated"<<endl;
		setDefaultState();
		detect_dog();
		setCurrentState();
		analyze_reach();
		setCurrentState();
		plan_path();
		setCurrentState();
	}
	else if( obj==_btn_im ){
		imager::getImage(_im_label);
	}
	else if( obj==_btn_scan ){
		imager::write2DImage();
	}
}

void SamplePlugin::timer()
{
	auto s = globals::states.size();
    if(0 <= _step && _step < s)
	{
		_slider->setMaximum(s - 1);
		_slider->setValue(_step);
		updatePlaybackState();
        _step++;
    }
	else
	{
		cout<<"finished playback"<<endl;
		_timer->stop();
		_step = 0;
	}
}

void SamplePlugin::setDefaultState()
{
	globals::states.clear();
	setCurrentState();
}

void SamplePlugin::setCurrentState()
{
	getRobWorkStudio()->setState(globals::getState());
}

void SamplePlugin::updatePlaybackState()
{
	auto s = globals::states.size();
	if(0 < _step && _step < s)
		getRobWorkStudio()->setState(globals::states[_step]);
}

void SamplePlugin::stateChangedListener(const State& state)
{
	globals::getState() = state;
}

