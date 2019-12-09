#include "SamplePlugin.hpp"
#include "globals.hpp"
#include "reachabilityAnalysis.hpp"
#include "rrtconnect.hpp"


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
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
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

Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}

bool playbackWasOn = false;
void SamplePlugin::onSliderMoved()
{
	// cout<<"slider moved"<<endl;
	_step = _slider->value();
	updatePlaybackState();
}
void SamplePlugin::onSliderPressed()
{
	// cout<<"slider pressed"<<endl;
	playbackWasOn = _timer->isActive();
	_timer->stop();
}
void SamplePlugin::onSliderReleased()
{
	// cout<<"slider released"<<endl;
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
	else if(obj==_btn0){
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
        rrtconnect::createPath(from, to, extend, maxTime);

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

void SamplePlugin::updatePlaybackState()
{
	getRobWorkStudio()->setState(globals::states[_step]);
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

void SamplePlugin::stateChangedListener(const State& state)
{
  globals::state = state;
}

