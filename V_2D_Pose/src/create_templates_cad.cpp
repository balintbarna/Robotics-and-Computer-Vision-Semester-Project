#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <covis/covis.h>
using namespace covis;
using namespace cv;

#include <cassert>
using namespace std;

#include <pcl/common/time.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace pcl;
using namespace io;
using namespace visualization;

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include <vtkCamera.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkPlatonicSolidSource.h>
#include <vtkPolyDataNormals.h>
#include <vtkSphereSource.h>
#include <vtkWindowToImageFilter.h>

// Types
typedef PointXYZ PointT;
typedef PointCloud<PointT> CloudT;

// Function to auto crop image (this version only outputs the ROI) - https://github.com/EndPointCorp/opencv_autocrop
Rect autocrop(Mat& src);

// Main
int main(int argc, const char** argv) {
    core::ProgramOptions po;
    po.addPositional("cad-file", "CAD file (must be in mesh format, e.g. PLY)");
    po.addOption("scale", 1, "apply this scale to the point coordinates of the input file - if your file is in e.g. mm, input 0.001 here");
    
    // Camera position options
    po.addOption("subdiv", 's', 1, "subdivisions of an icosahedron to generate a view sphere - the number of points become 10*4^subdiv + 2");
    
    po.addOption("radius", 'r', 1, "radius of the tessellated sphere, i.e. distance of the camera from the object - you can use multiple values here to get multiple radii");
    
    po.addOption("lat-begin", 0, "latitude start angle [deg] in [0,180] - 0 is at the pole (0,0,1)");
    po.addOption("lat-end", 180, "latitude end angle [deg] in [0,180]");
    
    po.addOption("lon-begin", 0, "longitude start angle [deg] in [0,360] - 0 is at the x-axis (1,0,0) and 180 is at (-1,0,0)");
    po.addOption("lon-end", 360, "longitude end angle [deg] in [0,360]");
    
    po.addOption("rotations", 't', 1, "number of in-plane rotations of the optical camera axis at each position (> 0)");
    po.addOption("rot-begin", 0, "rotation start angle [deg]");
    po.addOption("rot-end", 360, "rotation end angle [deg]");
    
    po.addOption("north", 'n', "0,0,1", "set a \"natural\" vertical axis for the model, defaults to the z-axis - the virtual camera's y-axis is aligned with the NEGATIVE of this axis");
    
    // Camera parameter options
    po.addOption("width", 640, "horizontal resolution");
    po.addOption("height", 480, "vertical resolution");
    po.addOption("fov", 49, "vertical field of view [degree]");
    
    // Output image options
    po.addOption("output-dir", ".", "output directory");
    po.addFlag("no-rgb", "don't output rgb templates");
    po.addFlag("no-depth", "don't output depth templates");
    po.addFlag("no-cloud", "don't output point cloud templates");
    po.addFlag("no-pose", "don't output camera poses");
    po.addFlag('c', "crop", "crop the output rgb/depth/cloud template");
    po.addFlag('a', "mask", "produce a crop mask for the output rgb/depth/cloud template");
    po.addFlag('m', "metric", "if this flag is set, the output depth image will be saved in the original metric units of the CAD file (float), otherwise it is scaled by 1000 and saved as 16-bit unsigned");
    po.addFlag('f', "filter-cloud", "remove NaNs, infintes and pure zeros from the cloud template - note that this destroys the organized structure");
    po.addOption("bc", "0.25,0.25,0.25", "background color for the RGB template (grayscale value or RGB triplet in [0,1])");
    po.addOption("bd", -1, "background \"depth\" for the point cloud template, use < 0 to set it to NaN - the depth template will in such case just have zero");
    po.addOption("aaframes", 1, "set the number of frames to use for anti-aliasing");
    
    // General options
    po.addFlag('v', "visualize", "show some visualizations");
    
    if(!po.parse(argc, argv))
        return 0;
    
    // Open CAD source as the first thing
    const string source = po.getValue("cad-file");
    const string ext = boost::algorithm::to_lower_copy(extension(boost::filesystem::path(source)));
    
    PolygonMesh mesh;
    COVIS_ASSERT(util::load(source, mesh));
    const float scale = po.getValue<float>("scale");
    COVIS_ASSERT(scale > 0);
    if(scale != 1) {
        cout << "NOTE: Scaling the input mesh by a factor of " << scale << "..." << endl;
        mesh = *filter::scale(boost::make_shared<PolygonMesh>(mesh), scale);
    }

    // Get options
    const size_t subdiv = po.getValue<size_t>("subdiv");
    const vector<double> radius = po.getVector<double>("radius");
    COVIS_ASSERT(!radius.empty() && radius[0] > 0 && subdiv > 0);
    const double latBegin = po.getValue<double>("lat-begin");
    const double latEnd = po.getValue<double>("lat-end");
    COVIS_ASSERT(latBegin >= 0 && latBegin <= 180 && latBegin <= latEnd);
    const double lonBegin = po.getValue<double>("lon-begin");
    const double lonEnd = po.getValue<double>("lon-end");
    COVIS_ASSERT(lonBegin >= 0 && lonBegin <= 360 && lonBegin <= lonEnd);
    const size_t rotations = po.getValue<size_t>("rotations");
    COVIS_ASSERT_MSG(rotations > 0, "At least one rotation must be performed to generate any templates!");
    const double rotBegin = po.getValue<double>("rot-begin");
    const double rotEnd = po.getValue<double>("rot-end");
    COVIS_ASSERT(rotBegin <= rotEnd);
    const vector<double> north = po.getVector<double>("north");
    COVIS_ASSERT(north.size() == 3);
    
    const size_t width = po.getValue<size_t>("width");
    const size_t height = po.getValue<size_t>("height");
    const double fov = po.getValue<double>("fov");
    
    const boost::filesystem::path outputDir = po.getValue("output-dir");
    if(!is_directory(outputDir))
        create_directory(outputDir);
    const bool noRgb = po.getFlag("no-rgb");
    const bool noDepth = po.getFlag("no-depth");
    const bool noCloud = po.getFlag("no-cloud");
    const bool noPose = po.getFlag("no-pose");
    const bool crop = po.getFlag("crop");
    const bool mask = po.getFlag("mask");
    const bool metric = po.getFlag("metric");
    const bool filterCloud = po.getFlag("filter-cloud");
    vector<double> bc = po.getVector<double>("bc");
    COVIS_ASSERT(bc.size() == 1 || bc.size() == 3);
    const double bd = po.getValue<double>("bd");
    const int aaframes = po.getValue<int>("aaframes");
    const bool visualize = po.getFlag("visualize");

    // Tessellate the view sphere by subdividing an icosahedron
    cout << "Tessellating icosahedron (12 vertices)  with " << subdiv << " subdivisions..." << endl;
    vtkSmartPointer<vtkPlatonicSolidSource> ico = vtkPlatonicSolidSource::New();
    ico->SetSolidTypeToIcosahedron();
    ico->Update();

    vtkSmartPointer<vtkLinearSubdivisionFilter> sub = vtkLinearSubdivisionFilter::New();
    sub->SetNumberOfSubdivisions(subdiv);
#if VTK_MAJOR_VERSION < 6
    sub->SetInput(ico->GetOutput());
#else
    sub->SetInputData(ico->GetOutput());
#endif
    sub->Update();
    
    vtkSmartPointer<vtkPoints> vtkpoints = sub->GetOutput()->GetPoints();
    
    cout << "\tGot " << vtkpoints->GetNumberOfPoints() << " vertices on the view sphere" << endl;

    cout << "Selecting camera positions within specified radius and lat-lon intervals and applying rotations..." << endl;
    cout << "\tRadii: ";
    core::print(radius, cout, "[" ,"]", ",");
    cout << "\tLatitude interval: [" << latBegin << "," << latEnd << "]" << endl;
    cout << "\tLongitude interval: [" << lonBegin << "," << lonEnd << "]" << endl;
    cout << "\tRotation(s) at each camera: " << rotations << " over [" << rotBegin << "," << rotEnd << "]" << endl;

    // Object center
    Eigen::Vector4f centroid;
    CloudT tmp;
    pcl::fromPCLPointCloud2(mesh.cloud, tmp);
    pcl::compute3DCentroid(tmp, centroid);

    // Generated camera positions and template labels
    vector<Eigen::Affine3f> Tcam;
    vector<string> labels;
    
    // Loop over all radii
    for(vector<double>::const_iterator itrad = radius.begin(); itrad != radius.end(); ++itrad) {
        // Loop over all sphere points and generate camera poses
        for(vtkIdType i = 0; i < vtkpoints->GetNumberOfPoints(); ++i) {
            // Get sampled camera position
            double pi[3];
            vtkpoints->GetPoint(i, pi);
            
            // Convert to  Eigen, normalize to 1 to put it on the unit sphere
            Eigen::Affine3f T;
            T.translation() << pi[0], pi[1], pi[2];
            T.translation().normalize();
            
            // Compute (lat,long) coordinates
            const double lati = acos(T.translation()[2]); // Relative angle with north pole in [0,pi]
            double longi = atan2(T.translation()[1], T.translation()[0]); // In [-pi,pi]
            if(longi < 0)
            	longi += 2 * M_PI; // In [0,2pi]
            
            // Now discard if not inside the specified lat-long interval
            const float tol = 1e-5f;
            if(lati < (latBegin - tol) * M_PI / 180 ||
                    lati > (latEnd + tol) * M_PI / 180 ||
                    longi < (lonBegin - tol) * M_PI / 180 ||
                    longi > (lonEnd + tol) * M_PI / 180) {
                continue;
            }
            
            // Generate a rotation, with the aim of pointing the y-axis in the opposite direction as the input pole
            Matrix3f R;
            R.col(2) = -T.translation(); // Z-axis points towards focal point

            // North pole, unity
            Vector3f vnorth = Vector3f(north[0], north[1], north[2]).normalized();
            
            // If z-axis is (anti-)parallel with north pole, skip
            const float ndot = R.col(2).dot(vnorth);
            const bool isNorth = fabsf(ndot) < 1 + 1e-5f && fabsf(ndot) > 1 - 1e-5f;
            if(isNorth)
                cout << "Frame with index " << i << " (anti-) parallel with north pole, using arbitrary camera orientation..." << endl;
            
            // Project to a sphere with specified radius
            T.translation() *= *itrad;

            // Move the whole sphere so the focal point is at the object center
            T.translation() += centroid.head<3>();

            if(isNorth) { // Use an arbitrary up-vector
                R.col(1) = R.col(2).unitOrthogonal();
            } else { // Use a well-defined up-vector (pointing north)
                // North pole
                vnorth *= *itrad;
                // Direction vector from camera to north pole
                const Vector3f dirCamNorth = (vnorth - T.translation()).normalized();
                // Projection of that vector to image plane
                const Vector3f dirCamNorthProj = dirCamNorth - dirCamNorth.dot(R.col(2)) * R.col(2);
                // That projection negated makes the up vector (y-axis points down in image)
                R.col(1) = -dirCamNorthProj.normalized();
            }

            // Final image axis
            R.col(0) = R.col(1).cross(R.col(2));
            T.linear() = R;

            // Generate all rotations around the view vector and push back the resulting camera pose and a label
            for(size_t j = 0; j < rotations; ++j) {
                // Generate a z-axis rotation matrix of the current angle (converted to radians)
                Matrix3f Rz = AngleAxisf((rotBegin + j * (rotEnd - rotBegin) / rotations) * M_PI / 180.0f, Vector3f::UnitZ()).matrix();
                // Apply rotation to the original camera pose
                Eigen::Affine3f Tz;
                Tz.translation() = T.translation();
                Tz.linear() = T.linear() * Rz;
                
                Tcam.push_back(Tz);
                ostringstream oss;
                oss << "template" << setfill('0') << setw(4) << labels.size();
                labels.push_back(oss.str());
            }
        }
    } // End loop over radii
    
    cout << "Got " << Tcam.size() << " camera positions!" << endl;
    
    // Show camera poses if visualization flag is on
    if(visualize) {
        cout << "Showing the camera positions..." << endl;
        PCLVisualizer visu("Camera frames");
        visu.addCoordinateSystem(radius[0] / 2);
        if(bc.size() == 1)
            visu.setBackgroundColor(bc[0], bc[0], bc[0]);
        else
            visu.setBackgroundColor(bc[0], bc[1], bc[2]);
        visu.addPolygonMesh(mesh);
        for(size_t i = 0; i < Tcam.size(); ++i)
            visu.addCoordinateSystem(radius[0]/10, Tcam[i], labels[i]);
        visu.spin();
    }
    
    // Print info on templates
    cout << "Generating " << Tcam.size() << " templates..." << endl;
    
    if(crop)
        cout << "\tNOTE: Cropping is enabled!" << endl;
    if(mask)
        cout << "\tNOTE: Mask images are output, which can be used for cropping the outputs!" << endl;
    if(filterCloud)
        cout << "\tNOTE: Cloud templates are filtered for NaNs, infinities and pure zeros - organized structure is lost!" << endl;
    
    if(!noDepth) {
        if(metric)
            cout << "\tNOTE: Metric units enabled - depth maps will be in same units as inputs, and the images will be saved as floats!" << endl;
        else
            cout << "\tNOTE: Metric units disabled - depth maps will be scaled by 1000, and the images will be saved as 16-bit unsigned - make sure that your input CAD file is scaled to meters!" << endl;
    }

    // Create a window for rendering templates from the mesh
    PCLVisualizer visu;
    visu.setShowFPS(false);
    if(bc.size() == 1)
        visu.setBackgroundColor(bc[0], bc[0], bc[0]);
    else
        visu.setBackgroundColor(bc[0], bc[1], bc[2]);
    visu.setSize(width, height);
    visu.setCameraFieldOfView(fov * M_PI / 180.0);
    visu.addPolygonMesh(mesh, "mesh");
    
    /*
     * TODO: Set some relevant parameters for rendering
     */
    vtkSmartPointer<vtkLODActor> actor = (*visu.getCloudActorMap())["mesh"].actor;
    vtkSmartPointer<vtkMapper> mapper = actor->GetMapper();
    vtkSmartPointer<vtkPolyData> poly = dynamic_cast<vtkPolyData*>(mapper->GetInput());
    
    vtkSmartPointer<vtkProperty> prop = actor->GetProperty();
    prop->SetInterpolationToFlat();
    prop->SetSpecularPower(128); // Full specularity to increase edge contrasts
    
    /*
     * Now start generating and saving templates using the visu instance
     */
    if(aaframes > 1) {
    	cout << "Using " << aaframes << " frames for anti-aliasing!" << endl;
    	visu.getRenderWindow()->SetAAFrames(aaframes); // Avoid aliasing
    }
    for(size_t i = 0; i < Tcam.size(); ++i) {
        // Setup
        visu.setWindowName(labels[i]);
        visu.setCameraPosition(
                Tcam[i].translation()[0], Tcam[i].translation()[1], Tcam[i].translation()[2], // Camera position
                centroid[0], centroid[1], centroid[2], // Focal point
                -Tcam[i].linear()(0,1), -Tcam[i].linear()(1,1), -Tcam[i].linear()(2,1) // Up vector (y)
                );
        visu.spinOnce(10, true);
        
        // RGB image
        vtkSmartPointer<vtkWindowToImageFilter> win2rgb = vtkWindowToImageFilter::New();
        win2rgb->SetInput(visu.getRenderWindow());
        win2rgb->ReadFrontBufferOff();


        win2rgb->SetInputBufferTypeToRGB();
        win2rgb->Update();
        vtkSmartPointer<vtkImageData> rgb = win2rgb->GetOutput();
        
        Mat_<Vec3b> imgrgb(height, width, reinterpret_cast<Vec3b*>(rgb->GetScalarPointer()));
        cvtColor(imgrgb, imgrgb, CV_RGB2BGR);
        
        // Z buffer
        Mat_<float> imgz(height, width);
        visu.getRenderWindow()->GetZbufferData(0, 0, width-1, height-1, imgz[0]);
        
        // Create real depth from z buffer (taken from PCLVisualizer source code for renderView())
        vtkRenderer* ren = visu.getRenderWindow()->GetRenderers()->GetFirstRenderer();
        vtkCamera* cam = ren->GetActiveCamera();

        // TODO: What are these matrices?
        vtkSmartPointer<vtkMatrix4x4> proj =
                cam->GetCompositeProjectionTransformMatrix(ren->GetTiledAspectRatio(), 0, 1);
        vtkSmartPointer<vtkMatrix4x4> view = cam->GetViewTransformMatrix();

        Matrix4f mat1, mat2;
        for(int r = 0; r < 4; ++r) {
            for(int c = 0; c < 4; ++c) {
                mat1(r, c) = proj->Element[r][c];
                mat2(r, c) = view->Element[r][c];
            }
        }
        mat1 = mat1.inverse().eval();
        
        // TODO: What are these?
        const float dwidth = 2.0f / float (width);
        const float dheight = 2.0f / float (height);
        
        // Convert to depth image and point cloud, both as float in original input metric units
        Mat_<float> imgdepth(height, width);
        PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>(width, height));
        for(size_t y = 0; y < height; ++y) {
            for(size_t x = 0; x < width; ++x) {
                // Output point
                PointXYZRGBA& p = (*cloud)(x,y);
                
                // Max z buffer value, background
                if(imgz(y, x) == 1.0) {
                    if(bd < 0) {
                        p.x = p.y = p.z = numeric_limits<float>::quiet_NaN();
                        imgdepth(y,x) = 0;
                    } else {
                        p.x = p.y = p.z = bd;
                        imgdepth(y,x) = bd;
                    }
                    continue;
                }
                
                Vector4f xyz(dwidth * x - 1.0f, dheight * y - 1.0f, imgz(y, x), 1.0f);
                xyz = mat2 * mat1 * xyz;

                const float w3 = 1.0f / xyz[3];
                xyz[0] *= w3;
                // Go from VTK frame to correct frame
                xyz[1] *= -w3;
                xyz[2] *= -w3;
                
                imgdepth(y,x) = xyz[2];

                p.x = xyz[0];
                p.y = xyz[1];
                p.z = xyz[2];
                p.r = imgrgb(y,x)[2];
                p.g = imgrgb(y,x)[1];
                p.b = imgrgb(y,x)[0];
            }
        }
        
        // Convert depth image to mm, 16-bit unsigned
        Mat_<ushort> imgdepthmm;
        if(!metric)
            imgdepth.convertTo(imgdepthmm, CV_16U, 1000, 0);
        
        // Cropping: if cropping enabled, crop all templates, otherwise output a mask that can be used to crop
        Mat_<uchar> cropMask;
        Rect win;
        if(crop || mask) {
            cropMask = Mat_<uchar>(height, width, uchar(0));
            win = autocrop(imgrgb);
            cropMask(win) = 255;
        }
        
        if(crop) {
            // Crop images
            imgrgb = imgrgb(win);
            if(metric)
            	imgdepth = imgdepth(win);
            else
                imgdepthmm = imgdepthmm(win);
            
            // Crop point cloud
            PointCloud<PointXYZRGBA> cloudc(win.width, win.height);
            for(int x = 0; x < win.width; ++x)
                for(int y = 0; y < win.height; ++y)
                    cloudc(x, y) = (*cloud)(win.x + x, win.y + y);
            *cloud = cloudc;
        }
        
        // Flip up/down because we want z pointing out and not in as in VTK
        flip(imgrgb, imgrgb, 0);
        if(metric)
        	flip(imgdepth, imgdepth, 0);
        else
            flip(imgdepthmm, imgdepthmm, 0);
        PointCloud<PointXYZRGBA> cloudf(cloud->width, cloud->height);
        for(size_t x = 0; x < cloud->width; ++x)
            for(size_t y = 0; y < cloud->height; ++y)
                cloudf(x, y) = (*cloud)(x, cloud->height - y - 1);
        *cloud = cloudf;

        // Filter NaNs, Infs and 0s
        if(filterCloud) {
            PointCloud<PointXYZRGBA> cloudf;
            for(size_t i = 0; i < cloud->size(); ++i)
                if(pcl::isFinite(cloud->points[i]) && !cloud->points[i].getVector3fMap().isZero())
                    cloudf.push_back(cloud->points[i]);
            *cloud = cloudf;
        }
        
        // Output to files
        if(!noRgb)
            COVIS_ASSERT(imwrite(outputDir.string() + "/" + labels[i] + ".png", imgrgb));
        if(!noDepth) {
            if(metric) {
                COVIS_ASSERT(imwrite(outputDir.string() + "/" + labels[i] + "_depth.png", imgdepth));
            } else {
                COVIS_ASSERT(imwrite(outputDir.string() + "/" + labels[i] + "_depth.png", imgdepthmm));
            }
        }
        if(!noCloud)
            COVIS_ASSERT(savePCDFile(outputDir.string() + "/" + labels[i] + "_cloud.pcd", *cloud, true) == 0);
        if(mask)
            COVIS_ASSERT(imwrite(outputDir.string() + "/" + labels[i] + "_mask.png", cropMask));
        if(!noPose)
            for(size_t i = 0; i < Tcam.size(); ++i)
                util::saveEigen(outputDir.string() + "/" + labels[i] + "_pose.txt",
                                Matrix4f(Tcam[i].inverse().matrix()),
                                true, // Row-major
                                false, // Binary
                                false); // Append
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

Rect autocrop(Mat& src) {
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

