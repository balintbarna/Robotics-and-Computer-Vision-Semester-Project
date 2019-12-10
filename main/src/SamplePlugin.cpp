#include "SamplePlugin.hpp"
#include "globals.hpp"
#include "reachabilityAnalysis.hpp"
#include "rrtconnect.hpp"
#include "imager.hpp"


SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// connect(_slider, SIGNAL(valueChanged(int)), this, SLOT(onSliderMoved()));
	connect(_slider, SIGNAL(sliderMoved(int)), this, SLOT(onSliderMoved()));
	connect(_slider, SIGNAL(sliderPressed()), this, SLOT(onSliderPressed()));
	connect(_slider, SIGNAL(sliderReleased()), this, SLOT(onSliderReleased()));

	// now connect stuff from the ui component
	connect(_btn_reach    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_state_playback    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_reset_playback    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_im    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_scan    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_calcpath    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

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
	else if(obj == _btn_reach)
	{
		cout<<"reach"<<endl;
		reach::analyse_reachability(globals::wc, globals::robot, globals::dogmiddle, globals::doghead, globals::detector, globals::goal, _spinBox->value());
	}
	else if(obj==_btn_calcpath){
		cout<<"preparing for calculating path"<<endl;
		// pause playback
        _timer->stop();
		// set up state
		setDefaultState();
		globals::gripper->setQ(Q(1, 0.045), globals::state);
		globals::robotRef->moveTo(reach::bestRobotPose, globals::state);
		Q from = reach::bestPickupConf;
		Q to = reach::bestPlaceConf;
		// rw::invkin::ClosedFormIKSolverUR::Ptr solver = ownedPtr(new rw::invkin::ClosedFormIKSolverUR(globals::robot, globals::state));
		// auto toTarget = rrtconnect::relativeTransformCalc(globals::goal, globals::state);
		// Q to = solver->solve(toTarget, globals::state)[0];
		globals::robot->setQ(from, globals::state);
		Kinematics::gripFrame(globals::dog.get(), globals::graspTcp.get(), globals::state);
		setCurrentState();
        rw::math::Math::seed();
		cout<<"calculating path"<<endl;
        rrtconnect::test_rrt(from, to);
		cout<<"path calculated"<<endl;

	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
	else if( obj==_btn_im ){
		imager::getImage(_im_label);
	}
	else if( obj==_btn_scan ){
		imager::get25DImage();
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
	globals::state = globals::wc->getDefaultState();
	setCurrentState();
}

void SamplePlugin::setCurrentState()
{
	getRobWorkStudio()->setState(globals::state);
}

void SamplePlugin::updatePlaybackState()
{
	auto s = globals::states.size();
	if(0 < _step && _step < s)
		getRobWorkStudio()->setState(globals::states[_step]);
}

void SamplePlugin::stateChangedListener(const State& state)
{
	globals::state = state;
}

