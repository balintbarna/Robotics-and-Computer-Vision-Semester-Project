#ifndef IMAGER_FUNCTIONS_HPP
#define IMAGER_FUNCTIONS_HPP

// OpenCV 3
#include <opencv2/opencv.hpp>

#include "globals.hpp"
#include "config.h"

namespace imager
{
    using namespace cv;
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

    Mat toOpenCVImage(const Image& img) {
        Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
        res.data = (uchar*)img.getImageData();
        return res;
    }

    vector<rw::geometry::PointCloud> get25DImage()
    {
        vector<rw::geometry::PointCloud> clouds;
        if (globals::framegrabber25D != NULL)
        {
            for(size_t i = 0; i < globals::cameras25D.size(); i++)
            {
                // Get the image as a RW image
                Frame* cameraFrame25D = globals::wc->findFrame(globals::cameras25D[i]); // "Camera");
                globals::framegrabber25D->grab(cameraFrame25D, globals::state);

                clouds.push_back(globals::framegrabber25D->getImage());
            }
        }
        return clouds;
    }

    void write2DImage()
    {
        vector<rw::geometry::PointCloud> clouds = get25DImage();


        for(int i = 0; i < clouds.size(); i++)
        {       
            string path(PATH_GENERATE);
            path.append(globals::cameras25D[i] + ".pcd");
            std::ofstream output(path);
            output << "# .PCD v.5 - Point Cloud Data file format\n";
            output << "FIELDS x y z\n";
            output << "SIZE 4 4 4\n";
            output << "TYPE F F F\n";
            output << "WIDTH " << clouds[i].getWidth() << "\n";
            output << "HEIGHT " << clouds[i].getHeight() << "\n";
            output << "POINTS " << clouds[i].getData().size() << "\n";
            output << "DATA ascii\n";
            for(const auto &p_tmp : clouds[i].getData())
            {
                rw::math::Vector3D<float> p = p_tmp;
                output << p(0) << " " << p(1) << " " << p(2) << "\n";
            }
            output.close();
        }
    }

    void getImage(QLabel *label)
    {
        if (globals::framegrabber != NULL)
        {
            for(size_t i = 0; i < globals::cameras.size(); i++)
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

                string path(PATH_GENERATE);
                path.append(globals::cameras[i] + ".png");
                cv::imwrite(path, imflip_mat);

                // Show in QLabel
                QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
                QPixmap p = QPixmap::fromImage(img);
                unsigned int maxW = 480;
                unsigned int maxH = 640;
                label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
            }
        }
    }
}

#endif /*IMAGER_FUNCTIONS_HPP*/