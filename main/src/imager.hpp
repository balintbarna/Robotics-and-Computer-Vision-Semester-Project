// OpenCV 3
#include <opencv2/opencv.hpp>

#include "globals.hpp"

namespace imager
{
    using namespace cv;

    Mat toOpenCVImage(const Image& img) {
        Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
        res.data = (uchar*)img.getImageData();
        return res;
    }

    void get25DImage()
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

    void getImage(QLabel *label)
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
                label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
            }
        }
    }
}