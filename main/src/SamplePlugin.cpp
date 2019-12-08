#include "SamplePlugin.hpp"
#include "globals.hpp"
#include "reachabilityAnalysis.hpp"



SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
	_stateTimer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));
    connect(_stateTimer, SIGNAL(timeout()), this, SLOT(stateTimer()));

	connect(_slider, SIGNAL(valueChanged(int)), this, SLOT(onSlide()));

	// now connect stuff from the ui component
	connect(_btn_reach    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_reach_all    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_state_playback    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_reset_playback    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_im    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_scan    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
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
	_stateTimer->stop();
    // Remove the texture render
	globals::close(this);
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}

void SamplePlugin::onSlide()
{
	_step = _slider->value();
}

void SamplePlugin::btnPressed() {
    QObject *obj = sender();
	if(obj == _btn_state_playback)
	{
		if (!_stateTimer->isActive())
		{
			cout<<"starting playback"<<endl;
            _stateTimer->start(50); // ms
        }
        else
		{
			cout<<"stopping playback"<<endl;
			_stateTimer->stop();
		}
	}
	if(obj == _btn_reset_playback)
	{
		cout<<"resetting playback"<<endl;
		_stateTimer->stop();
		_step = 0;
		getRobWorkStudio()->setState(globals::states[_step]);
	}
	if(obj == _btn_reach)
	{
		cout<<"reach"<<endl;
		analyse_reachability(globals::wc, globals::robot, globals::dogmiddle, globals::doghead, globals::detector, false, globals::goal);
	}
	if(obj == _btn_reach_all)
	{
		cout<<"reach all"<<endl;
		analyse_reachability(globals::wc, globals::robot, globals::dogmiddle, globals::doghead, globals::detector, true, globals::goal);
	}
	if(obj==_btn0){
//		log().info() << "Button 0\n";
//		// Toggle the timer on and off
//		if (!_timer25D->isActive())
//		    _timer25D->start(100); // run 10 Hz
//		else
//			_timer25D->stop();
        _timer->stop();
        rw::math::Math::seed();
        double extend = 0.05;
        double maxTime = 60;
        Q from(6, 1.571, -1.572, -1.572, -1.572, 1.571, 0);
        Q to(6, 1.847, -2.465, -1.602, -0.647, 1.571, 0); //From pose estimation
        createPathRRTConnect(from, to, extend, maxTime);


	} else if(obj==_btn1){
        log().info() << "Button 1\n";
        // Toggle the timer on and off
        if (!_timer->isActive()){
            _timer->start(100); // ms
            _step = 0;
        }
        else
            _step = 0;

	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
	else if( obj==_btn_im ){
		getImage();
	}
	else if( obj==_btn_scan ){
		get25DImage();
	}
}

void SamplePlugin::stateTimer()
{
	auto s = globals::states.size();
    if(0 <= _step && _step < s)
	{
		_slider->setMaximum(s);
		_slider->setValue(_step);
        getRobWorkStudio()->setState(globals::states[_step]);
        _step++;
    }
	else
	{
		cout<<"finished playback"<<endl;
		_stateTimer->stop();
		_step = 0;
	}
}


void SamplePlugin::get25DImage()
{
	if (globals::framegrabber25D != NULL)
	{
		for( int i = 0; i < globals::cameras25D.size(); i ++)
		{
			// Get the image as a RW image
			Frame* cameraFrame25D = globals::wc->findFrame(globals::cameras25D[i]); // "Camera");
			globals::framegrabber25D->grab(cameraFrame25D, globals::state);

			//const Image& image = _framegrabber->getImage();

			const rw::geometry::PointCloud* img = &(globals::framegrabber25D->getImage());

			std::ofstream output(globals::cameras25D[i] + ".pcd");
			output << "# .PCD v.5 - Point Cloud Data file format\n";
			output << "FIELDS x y z\n";
			output << "SIZE 4 4 4\n";
			output << "TYPE F F F\n";
			output << "WIDTH " << img->getWidth() << "\n";
			output << "HEIGHT " << img->getHeight() << "\n";
			output << "POINTS " << img->getData().size() << "\n";
			output << "DATA ascii\n";
            for(const auto &p_tmp : img->getData())
			{
				rw::math::Vector3D<float> p = p_tmp;
				output << p(0) << " " << p(1) << " " << p(2) << "\n";
			}
			output.close();

		}
	}
}

void SamplePlugin::getImage()
{
	if (globals::framegrabber != NULL)
	{
		for( int i = 0; i < globals::cameras.size(); i ++)
		{
			// Get the image as a RW image
			Frame* cameraFrame = globals::wc->findFrame(globals::cameras[i]); // "Camera");
			globals::framegrabber->grab(cameraFrame, globals::state);

			const rw::sensor::Image* rw_image = &(globals::framegrabber->getImage());

			// Convert to OpenCV matrix.
			cv::Mat image = cv::Mat(rw_image->getHeight(), rw_image->getWidth(), CV_8UC3, (rw::sensor::Image*)rw_image->getImageData());

			// Convert to OpenCV image
			Mat imflip, imflip_mat;
			cv::flip(image, imflip, 1);
			cv::cvtColor( imflip, imflip_mat, COLOR_RGB2BGR );

			cv::imwrite(globals::cameras[i] + ".png", imflip_mat );

			// Show in QLabel
			QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
			QPixmap p = QPixmap::fromImage(img);
			unsigned int maxW = 480;
			unsigned int maxH = 640;
			_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
		}
	}
}

void SamplePlugin::timer()
{
    if(0 <= _step && _step < globals::path.size())
	{
        globals::robot->setQ(globals::path.at(_step),globals::state);
        getRobWorkStudio()->setState(globals::state);
        _step++;
    }
	else
	{
		_timer->stop();
	}
	
}

void SamplePlugin::stateChangedListener(const State& state)
{
  globals::state = state;
}

bool SamplePlugin::checkCollisions(Device::Ptr robot, const State &state, const CollisionDetector &detector, const Q &q)
{
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    robot->setQ(q,testState);
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

void SamplePlugin::createPathRRTConnect(Q from, Q to,  double extend, double maxTime)
{
    globals::robot->setQ(from,globals::state);
    getRobWorkStudio()->setState(globals::state);
    CollisionDetector detector(globals::wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,globals::robot,globals::state);
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(globals::robot),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    globals::path.clear();
    if (!checkCollisions(globals::robot, globals::state, detector, from))
        cout << from << " is in colission!" << endl;
    if (!checkCollisions(globals::robot, globals::state, detector, to))
        cout << to << " is in colission!" << endl;;
    Timer t;
    t.resetAndResume();
    planner->query(from,to,globals::path,maxTime);
    t.pause();


    if (t.getTime() >= maxTime) {
        cout << "Notice: max time of " << maxTime << " seconds reached." << endl;
    }

	const int duration = 10;

    if(globals::path.size() == 2){  //The interpolated path between Q start and Q goal is collision free. Set the duration with respect to the desired velocity
        LinearInterpolator<Q> linInt(from, to, duration);
        QPath tempQ;
        for(int i = 0; i < duration+1; i++){
            tempQ.push_back(linInt.x(i));
        }

        globals::path=tempQ;
    }
}
