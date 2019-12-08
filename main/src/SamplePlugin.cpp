#include "SamplePlugin.hpp"
#include "globals.hpp"
#include "reachabilityAnalysis.hpp"



SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn_reach    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
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
    globals::wc = workcell;
    globals::state = globals::wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (globals::wc != NULL) {
	// Add the texture render to this workcell if there is a frame for texture
	Frame* textureFrame = globals::wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",globals::textureRender,textureFrame);
	}
	// Add the background render to this workcell if there is a frame for texture
	Frame* bgFrame = globals::wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",globals::bgRender,bgFrame);
	}

	// Create a GLFrameGrabber if there is a camera frame with a Camera property set
	Frame* cameraFrame = globals::wc->findFrame(globals::cameras[0]);
	if (cameraFrame != NULL) {
		if (cameraFrame->getPropertyMap().has("Camera")) {
			// Read the dimensions and field of view
			double fovy;
			int width,height;
			std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
			std::istringstream iss (camParam, std::istringstream::in);
			iss >> fovy >> width >> height;
			// Create a frame grabber
			globals::framegrabber = new GLFrameGrabber(width,height,fovy);
			SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
			globals::framegrabber->init(gldrawer);
		}
	}
	
	Frame* cameraFrame25D = globals::wc->findFrame(globals::cameras25D[0]);
	if (cameraFrame25D != NULL) {
		if (cameraFrame25D->getPropertyMap().has("Scanner25D")) {
			// Read the dimensions and field of view
			double fovy;
			int width,height;
			std::string camParam = cameraFrame25D->getPropertyMap().get<std::string>("Scanner25D");
			std::istringstream iss (camParam, std::istringstream::in);
			iss >> fovy >> width >> height;
			// Create a frame grabber
			globals::framegrabber25D = new GLFrameGrabber25D(width,height,fovy);
			SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
			globals::framegrabber25D->init(gldrawer);
		}
	}

    globals::device = globals::wc->findDevice<SerialDevice>("UR-6-85-5-A");
    _step = -1;
	globals::detector = ownedPtr(new CollisionDetector(globals::wc, ProximityStrategyFactory::makeDefaultCollisionStrategy()));
	globals::dog = globals::wc->findFrame<MovableFrame>("Dog");
	globals::target = globals::wc->findFrame<MovableFrame>("Target");
    }
}


void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
	Frame* textureFrame = globals::wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = globals::wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (globals::framegrabber != NULL) {
		delete globals::framegrabber;
	}
	globals::framegrabber = NULL;
	globals::wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}


void SamplePlugin::btnPressed() {
    QObject *obj = sender();
	if(obj == _btn_reach)
	{
		cout<<"reach"<<endl;
		analyse_reachability(getRobWorkStudio(), globals::wc, globals::device, globals::target, globals::detector);
		if (!_timer->isActive()){
            _timer->start(100); // run 10 Hz
            _step = 0;
        }
        else
            _step = 0;
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
            _timer->start(100); // run 10 Hz
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


void SamplePlugin::get25DImage() {
	if (globals::framegrabber25D != NULL) {
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

void SamplePlugin::getImage() {
	if (globals::framegrabber != NULL) {
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

void SamplePlugin::timer() {
    if(0 <= _step && _step < globals::path.size()){
        globals::device->setQ(globals::path.at(_step),globals::state);
        getRobWorkStudio()->setState(globals::state);
        _step++;
    }
	else
	{
		_timer->stop();
	}
	
}

void SamplePlugin::stateChangedListener(const State& state) {
  globals::state = state;
}

bool SamplePlugin::checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
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

void SamplePlugin::createPathRRTConnect(Q from, Q to,  double extend, double maxTime){
    globals::device->setQ(from,globals::state);
    getRobWorkStudio()->setState(globals::state);
    CollisionDetector detector(globals::wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,globals::device,globals::state);
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(globals::device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    globals::path.clear();
    if (!checkCollisions(globals::device, globals::state, detector, from))
        cout << from << " is in colission!" << endl;
    if (!checkCollisions(globals::device, globals::state, detector, to))
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
