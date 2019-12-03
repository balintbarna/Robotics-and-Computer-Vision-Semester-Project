// Copyright (c) 2019, University of Southern Denmark
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the University of Southern Denmark nor the names of
//    its contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF SOUTHERN DENMARK BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include <covis/covis.h>
using namespace covis;

// OpenCV
// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd/linemod.hpp>

using namespace cv;

using namespace std;

inline Rect autocrop(Mat& src);

cv::linemod::Detector createLinemodDetector(int pyramid_depth)
{
  std::vector<int> pyramid;
  for(int i = 0; i < pyramid_depth; i++)
  {
    pyramid.push_back(pow(2,i));
  }

  std::vector< Ptr<cv::linemod::Modality> > modals;
  modals.push_back( cv::linemod::Modality::create ("ColorGradient") );
  cv::linemod::Detector detector( modals, pyramid );
  return detector;
}

/*
 * Main entry point
 */
int main(int argc, const char** argv) {
  using namespace std;
  // Setup program options
  core::ProgramOptions po;
  //po.addPositional("template", "template image(s)");
  po.addPositional("template", "folder containing template image(s) and pose(s)");
  po.addPositional("image", "test image(s)");
    
  po.addOption("threshold", 't', 10, "if positive, accept all detections up to this threshold");
  po.addOption("pyramid_depth", 'p', 3, "levels of pyramid");

  // Parse
  if(!po.parse(argc, argv))
    return 1;
  //po.print();//because it is run in script
    
  const std::vector<std::string> ipath = po.getVector("image");

  const std::string tpath = po.getValue("template");
  
  const float threshold = po.getValue<float>("threshold");
  cout<<"threshold:"<<threshold<<endl;
  const int pyramid_depth = po.getValue<int>("pyramid_depth");
  cout<<"pyramid levels:"<<pyramid_depth<<endl;
  
  // Training data to be loaded for the 2D matcher
  std::vector<Mat> templates;
  std::vector<Eigen::Matrix4f> tposes;

  std::vector<cv::Point> offset;
  
  int cnt = 0; // Current template index

  cv::linemod::Detector detector = createLinemodDetector(pyramid_depth);

  while(true) {
    // Get RGB template
    char tfile[1024];
    sprintf(tfile, "/template%04i.png", cnt);
    Mat t = imread(tpath + std::string(tfile), IMREAD_UNCHANGED);

    if(t.empty())
      break;
    
    templates.push_back(t.clone());

    cv::Mat out;

    Rect win = autocrop(t);
	
    win.height = win.height + 4;
    win.width = win.width + 4;
    win.x = win.x - 2;
    win.y = win.y - 2;
	
    t = t(win);

    offset.push_back( cv::Point( win.width, win.height ));
    
    cv::inRange(t, cv::Scalar(0,0,244), cv::Scalar(1,1,255), out);
    out = 255 - out;
    
    cv::imshow("template", t );
    cv::imshow("mask", out );
    //cv::waitKey();
      
    std::vector<Mat> sources;
    sources.push_back(t.clone());

    sprintf(tfile, "%04i", cnt);

    // insert templates into the detectior
    detector.addTemplate(sources, std::string(tfile), out);

    cnt++;
  }    

  std::cout << "Number of templates: " << templates.size() << std::endl;

  for(int image_index = 0; image_index < int(ipath.size()); image_index++) {
    	
    Mat img = imread(ipath[image_index], IMREAD_UNCHANGED); // imread(po.getValue("image"), IMREAD_UNCHANGED);
    COVIS_ASSERT_MSG(!img.empty(), "Cannot read test image " << po.getValue("image") << "!");

    cv::Mat image = img.clone(); 
		    
    std::vector<Mat> sources;
    sources.push_back( image );

    std::vector< cv::linemod::Match > matches;

    detector.match( sources, threshold, matches );

    std::cout << "Number of matches: " << matches.size() << std::endl;
    if(matches.size() < 1)
    {
      std::cout<<"No matches, continuing..."<<std::endl;
      continue;
    }
    
    int i = 0;
    	    	   
    cv::imshow("temp", templates[ atoi(matches[i].class_id.c_str()) ]);

    circle(img, cv::Point( matches[i].x, matches[i].y), 8, cv::Scalar(0, 255, 0) , -1 );

    char pfile[1024];
    sprintf(pfile, "/template%04i_pose.txt",  atoi(matches[i].class_id.c_str()) );
    Eigen::Matrix4f m;
    covis::util::loadEigen(tpath + std::string(pfile), m);

    std::cout << m << std::endl;

    cv::imshow( "img", img );
    cv::waitKey(0);
  }
  
  return 0;
}


// Internal function used by autocrop()
inline bool isBorder(Mat& edge, Vec3b color) {
  Mat im = edge.clone().reshape(0,1);

  bool res = true;
  for(int i = 0; i < im.cols; ++i)
    res &= (color == im.at<Vec3b>(0,i));

  return res;
}

inline Rect autocrop(Mat& src) {
  COVIS_ASSERT(src.type() == CV_8UC3);
  Rect win(0, 0, src.cols, src.rows);

  vector<Rect> edges;
  edges.push_back(Rect(0, 0, src.cols, 1));
  edges.push_back(Rect(src.cols-2, 0, 1, src.rows));
  edges.push_back(Rect(0, src.rows-2, src.cols, 1));
  edges.push_back(Rect(0, 0, 1, src.rows));

  Mat edge;
  int nborder = 0;
  Vec3b color = src.at<Vec3b>(0,0);

  for (size_t i = 0; i < edges.size(); ++i) {
    edge = src(edges[i]);
    nborder += isBorder(edge, color);
  }

  if (nborder < 4)
    return win;

  bool next;

  do {
    edge = src(Rect(win.x, win.height-2, win.width, 1));
    if( (next = isBorder(edge, color)) )
      win.height--;
  } while (next && win.height > 0);

  do {
    edge = src(Rect(win.width-2, win.y, 1, win.height));
    if( (next = isBorder(edge, color)) )
      win.width--;
  } while (next && win.width > 0);

  do {
    edge = src(Rect(win.x, win.y, win.width, 1));
    if( (next = isBorder(edge, color)) )
      win.y++, win.height--;
  } while (next && win.y <= src.rows);

  do {
    edge = src(Rect(win.x, win.y, 1, win.height));
    if( (next = isBorder(edge, color)) )
      win.x++, win.width--;
  } while (next && win.x <= src.cols);
    
  return win;
}
