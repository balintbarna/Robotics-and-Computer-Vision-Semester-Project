#ifndef PCL_FUNCTIONS_HPP
#define PCL_FUNCTIONS_HPP

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

#include "imager.hpp"
#include "config.h"

namespace pointcloud
{
    using namespace std;
    using namespace pcl;
    typedef PointXYZ PointIn;
    typedef PointNormal PointT;
    typedef Histogram<153> FeatureT;

    // filtering parameters
    float plane_threshold = 0.02f;
    float voxel_size = 0.005f; // (distance between | size of) voxels
    float outlier_neighbour_mult = 10000.0f; // when searching for outliers, this times voxel_size is the radius of neighbours
    float smoothing_neighbour_mult = 2.0f; // when smoothing, this times voxel_size is the radius for neighbours that are used in smoothing
    float feature_neighbour_mult = 4.0f; // when computing features, this times voxel_size is the radious for neighbours included in feature
    float feature_radius = 0.2f;
    int neighbour_for_normal = 20; // this many neighbours are included in normal calculation

    float xmin = -0.3f, xmax = 0.3f;
    float ymin = -0.15f, ymax = 0.3f;
    float zmin = -1.2f, zmax = -0.7f;
    // Set RANSAC parameters
    const size_t global_ransac_iter = 2000; // global ransac does this many iterations
    const size_t local_ransac_iter = 200; // local ipc does this many iteractions
    const float dist_threshold_global = 0.02;
    const float dist_threshold_local = 0.01;
    const float thressq_global = dist_threshold_global * dist_threshold_global;
    const float thressq_local = dist_threshold_local * dist_threshold_local;

    #pragma region IO
    PointCloud<PointIn>::Ptr capture_pointcloud()
    {
        vector<rw::geometry::PointCloud> clouds = imager::get25DImage();
        rw::geometry::PointCloud &cloud = clouds[0];

        PointCloud<PointIn>::Ptr nucloud(new PointCloud<PointIn>);

        for(const auto &pixel : cloud.getData())
        {
            PointIn p(pixel(0), pixel(1), pixel(2));
            nucloud->push_back(p);
        }

        return nucloud;
    }

    PointCloud<PointIn>::Ptr load_object()
    {
        PointCloud<PointIn>::Ptr object(new PointCloud<PointIn>);
        pcl::PCDReader reader;
        string path(PATH_RESOURCE);
        path.append("object.pcd");
        reader.read(path, *object); // Remember to download the file first!

        return object;
    }
    #pragma endregion

    #pragma region FILTER
    void planeRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud)
	{
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (plane_threshold);
		seg.setInputCloud(input_cloud);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		}
		// pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(input_cloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*output_cloud);
	}

    void voxelGrid( PointCloud<PointIn>::Ptr input_cloud, PointCloud<PointIn>::Ptr &output_cloud )
    {
        VoxelGrid<PointIn> vg;
        vg.setInputCloud(input_cloud);
        vg.setLeafSize (voxel_size, voxel_size, voxel_size);
        vg.filter(*output_cloud);
    }

    void outlierRemoval( PointCloud<PointIn>::Ptr input_cloud, PointCloud<PointIn>::Ptr &output_cloud )
    {
        StatisticalOutlierRemoval<PointIn> sor;
        sor.setInputCloud(input_cloud);
        sor.setMeanK(30);
        sor.setStddevMulThresh(outlier_neighbour_mult*voxel_size);
        sor.filter(*output_cloud);
    }

    void spatialFilter( PointCloud<PointIn>::Ptr input_cloud, PointCloud<PointIn>::Ptr &output_cloud )
    {
        PassThrough<PointIn> filter_x;
		filter_x.setInputCloud(input_cloud);
		filter_x.setFilterFieldName("x");
		filter_x.setFilterLimits(xmin, xmax);
		filter_x.filter(*output_cloud);
			
		PassThrough<PointIn> filter_y;
		filter_y.setInputCloud(input_cloud);
		filter_y.setFilterFieldName("y");
		filter_y.setFilterLimits(ymin, ymax);
		filter_y.filter(*output_cloud);

		PassThrough<PointIn> filter_z;
		filter_z.setInputCloud(input_cloud);
		filter_z.setFilterFieldName("z");
		filter_z.setFilterLimits(zmin, zmax);
		filter_z.filter(*output_cloud);
    }

    void fake_smoothing(PointCloud<PointIn>::Ptr input_cloud, PointCloud<PointT>::Ptr &output_cloud)
    {
        for(auto &pixel : *input_cloud)
        {
            PointT nu;
            nu.x = pixel.x;
            nu.y = pixel.y;
            nu.z = pixel.z;
            output_cloud->push_back(nu);
        }
    }

    void smoothing(PointCloud<PointIn>::Ptr input_cloud, PointCloud<PointT>::Ptr &output_cloud)
    {
        search::KdTree<PointIn>::Ptr tree (new search::KdTree<PointIn>);
        MovingLeastSquares<PointIn, PointT> mls;

        // Set parameters
        // mls.setComputeNormals (true);
        mls.setInputCloud (input_cloud);
        mls.setPolynomialOrder (2);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (voxel_size*smoothing_neighbour_mult);
        mls.process (*output_cloud);
    }

    void show(std::string title, PointCloud<PointIn>::Ptr scene, PointCloud<PointIn>::Ptr object)
    {
        using namespace visualization;
        PCLVisualizer v(title);
	    v.addCoordinateSystem();
        v.addPointCloud<PointIn>(object, PointCloudColorHandlerCustom<PointIn>(object, 0, 255, 0), "object");
        v.addPointCloud<PointIn>(scene, PointCloudColorHandlerCustom<PointIn>(scene, 255, 0, 0), "scene");
        v.spin();
    }

    void show(std::string title, PointCloud<PointT>::Ptr scene, PointCloud<PointT>::Ptr object)
    {
        using namespace visualization;
        PCLVisualizer v(title);
	    v.addCoordinateSystem();
        v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0), "scene");
        v.spin();
    }

    void preprocess(PointCloud<PointIn>::Ptr input_scene, PointCloud<PointT>::Ptr &output_scene,
                    PointCloud<PointIn>::Ptr input_object, PointCloud<PointT>::Ptr &output_object)
    {
        PointIn minPt, maxPt;
        pcl::getMinMax3D(*input_scene, minPt, maxPt);
        std::cout << "Max x: " << maxPt.x << std::endl;
        std::cout << "Max y: " << maxPt.y << std::endl;
        std::cout << "Max z: " << maxPt.z << std::endl;
        std::cout << "Min x: " << minPt.x << std::endl;
        std::cout << "Min y: " << minPt.y << std::endl;
        std::cout << "Min z: " << minPt.z << std::endl;

        cout << "Scene before filtering: " 
            << input_scene->width * input_scene->height 
            << " data points (" << pcl::getFieldsList (*input_scene) << ")."
            <<endl;

        cout << "Object before filtering: " 
            << input_object->width * input_object->height 
            << " data points (" << pcl::getFieldsList (*input_object) << ")."
            <<endl;

        spatialFilter(input_scene, input_scene);
        // show("After spatial filter", input_scene, input_object);

        voxelGrid(input_scene, input_scene);
        voxelGrid(input_object, input_object);
        show("After voxeling", input_scene, input_object);

        planeRemoval(input_scene, input_scene);
        // show("After plane removal", input_scene, input_object);

        outlierRemoval(input_scene, input_scene);
        // show("After outlier removal", input_scene, input_object);

        // smoothing(input_scene, output_scene);
        // smoothing(input_object, output_object);
        fake_smoothing(input_scene, output_scene);
        fake_smoothing(input_object, output_object);
        // show("After smoothing", output_scene, output_object);

        std::cout << "Scene after filtering: " 
            << output_scene->width * output_scene->height 
            << " data points (" << pcl::getFieldsList (*output_scene) << ")." << endl;

        std::cout << "Object after filtering: " 
            << output_object->width * output_object->height 
            << " data points (" << pcl::getFieldsList (*output_object) << ")." << endl;
    }
    #pragma endregion

    #pragma region GLOBAL
    inline float dist_sq(const FeatureT& query, const FeatureT& target)
    {
        float result = 0.0;
        for(int i = 0; i < FeatureT::descriptorSize(); ++i)
        {
            const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
            result += diff * diff;
        }
        
        return result;
    }

    void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq)
    {
        idx = 0;
        distsq = dist_sq(query, target[0]);
        for(size_t i = 1; i < target.size(); ++i)
        {
            const float disti = dist_sq(query, target[i]);
            if(disti < distsq)
            {
                idx = i;
                distsq = disti;
            }
        }
    }

    void compute_normals(PointCloud<PointT>::Ptr &cloud)
    {
        ScopeTime t("Surface normals");
        NormalEstimation<PointT,PointT> ne;
        ne.setKSearch(neighbour_for_normal);
        
        ne.setInputCloud(cloud);
        ne.compute(*cloud);
    }

    void compute_feature(PointCloud<PointT>::Ptr cloud, PointCloud<FeatureT>::Ptr &features)
    {
        ScopeTime t("Shape features");
        
        SpinImageEstimation<PointT,PointT,FeatureT> spin;
        spin.setRadiusSearch(feature_radius);
        
        spin.setInputCloud(cloud);
        spin.setInputNormals(cloud);
        spin.compute(*features);
    }

    Correspondences compute_correspondances(PointCloud<FeatureT>::Ptr &scene_feature, PointCloud<FeatureT>::Ptr &object_feature)
    {
        ScopeTime t("Feature matches");
        Correspondences corr(object_feature->size());
        for(size_t i = 0; i < object_feature->size(); ++i) {
            corr[i].index_query = i;
            nearest_feature(object_feature->points[i], *scene_feature, corr[i].index_match, corr[i].distance);
        }
        return corr;
    }

    void show_matches(std::string title, PointCloud<PointT>::Ptr scene, PointCloud<PointT>::Ptr object, Correspondences &corr)
    {
        using namespace pcl::visualization;
        PCLVisualizer v(title);
        v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0), "scene");
        v.addCorrespondences<PointT>(object, scene, corr, 1);
        v.spin();
    }

    void global_ransac(search::KdTree<PointNormal> &tree, Eigen::Matrix4f &pose, PointCloud<PointT>::Ptr scene, PointCloud<PointT>::Ptr object, PointCloud<PointT>::Ptr object_aligned, Correspondences &corr)
    {
        using namespace pcl::common;
        using namespace pcl::io;
        using namespace pcl::registration;
        using namespace pcl::search;
        using namespace Eigen;
        float penalty = FLT_MAX;
        ScopeTime t("RANSAC");
        cout << "Starting RANSAC..." << endl;
        UniformGenerator<int> gen(0, corr.size() - 1);
        for(size_t i = 0; i < global_ransac_iter; ++i) {
            if((i + 1) % 100 == 0)
                cout << "\t" << i+1 << endl;
            // Sample 3 random correspondences
            vector<int> idxobj(3);
            vector<int> idxscn(3);
            for(int j = 0; j < 3; ++j) {
                const int idx = gen.run();
                idxobj[j] = corr[idx].index_query;
                idxscn[j] = corr[idx].index_match;
            }
            
            // Estimate transformation
            Matrix4f T;
            TransformationEstimationSVD<PointNormal,PointNormal> est;
            est.estimateRigidTransformation(*object, idxobj, *scene, idxscn, T);
            
            // Apply pose
            transformPointCloud(*object, *object_aligned, T);
            
            // Validate
            vector<vector<int> > idx;
            vector<vector<float> > distsq;
            tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
            
            // Compute inliers and RMSE
            size_t inliers = 0;
            float rmse = 0;
            for(size_t j = 0; j < distsq.size(); ++j)
                if(distsq[j][0] <= thressq_global)
                    ++inliers, rmse += distsq[j][0];
            rmse = sqrtf(rmse / inliers);
            
            // Evaluate a penalty function
            const float outlier_rate = 1.0f - float(inliers) / object->size();
            //const float penaltyi = rmse;
            const float penaltyi = outlier_rate;
            
            // Update result
            if(penaltyi < penalty) {
                cout << "\t--> Got a new model with " << inliers << " inliers!" << endl;
                penalty = penaltyi;
                pose = T;
            }
        }
    }

    void global_align(PointCloud<PointT>::Ptr scene, PointCloud<PointT>::Ptr object, PointCloud<PointT>::Ptr object_aligned, Eigen::Matrix4f &pose)
    {
        using namespace std;
        using namespace pcl;
        using namespace Eigen;
        PointCloud<FeatureT>::Ptr object_feature(new PointCloud<FeatureT>);
        PointCloud<FeatureT>::Ptr scene_feature(new PointCloud<FeatureT>);

        compute_normals(object);
        compute_normals(scene);

        compute_feature(object, object_feature);
        compute_feature(scene, scene_feature);

        auto corr = compute_correspondances(scene_feature, object_feature);

        show_matches("Matches", scene, object, corr);

        // Create a k-d tree for scene
        search::KdTree<PointNormal> tree;
        tree.setInputCloud(scene);
        
        // Start RANSAC
        global_ransac(tree, pose, scene, object, object_aligned, corr);
        
        transformPointCloud(*object, *object_aligned, pose);
        
        // Compute inliers and RMSE
        vector<vector<int> > idx;
        vector<vector<float> > distsq;
        tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
        size_t inliers = 0;
        float rmse = 0;
        for(size_t i = 0; i < distsq.size(); ++i)
            if(distsq[i][0] <= thressq_global)
                ++inliers, rmse += distsq[i][0];
        rmse = sqrtf(rmse / inliers);
    
        // Print pose
        cout << "Got the following pose:" << endl << pose << endl;
        cout << "Inliers: " << inliers << "/" << object->size() << endl;
        cout << "RMSE: " << rmse << endl;
    }
    #pragma endregion

    #pragma region LOCAL
    void local_align(PointCloud<PointT>::Ptr scene, PointCloud<PointT>::Ptr object, Eigen::Matrix4f &pose)
    {
        using namespace std;
        using namespace pcl;
        using namespace pcl::io;
        using namespace pcl::registration;
        using namespace pcl::search;
        using namespace pcl::visualization;
        using namespace Eigen;

        // Create a k-d tree for scene
        search::KdTree<PointNormal> tree;
        tree.setInputCloud(scene);
        
        // Start ICP
        {
            ScopeTime t("ICP");
            cout << "Starting ICP..." << endl;
            for(size_t i = 0; i < local_ransac_iter; ++i) {
                // 1) Find closest points
                vector<vector<int> > idx;
                vector<vector<float> > distsq;
                tree.nearestKSearch(*object, std::vector<int>(), 1, idx, distsq);
                
                // Threshold and create indices for object/scene and compute RMSE
                vector<int> idxobj;
                vector<int> idxscn;
                for(size_t j = 0; j < idx.size(); ++j) {
                    if(distsq[j][0] <= thressq_local) {
                        idxobj.push_back(j);
                        idxscn.push_back(idx[j][0]);
                    }
                }
                
                // 2) Estimate transformation
                Matrix4f T;
                TransformationEstimationSVD<PointNormal,PointNormal> est;
                est.estimateRigidTransformation(*object, idxobj, *scene, idxscn, T);
                
                // 3) Apply pose
                transformPointCloud(*object, *object, T);
                
                // 4) Update result
                pose = T * pose;
            }
            
            // Compute inliers and RMSE
            vector<vector<int> > idx;
            vector<vector<float> > distsq;
            tree.nearestKSearch(*object, std::vector<int>(), 1, idx, distsq);
            size_t inliers = 0;
            float rmse = 0;
            for(size_t i = 0; i < distsq.size(); ++i)
                if(distsq[i][0] <= thressq_local)
                    ++inliers, rmse += distsq[i][0];
            rmse = sqrtf(rmse / inliers);
        
            // Print pose
            cout << "Got the following pose:" << endl << pose << endl;
            cout << "Inliers: " << inliers << "/" << object->size() << endl;
            cout << "RMSE: " << rmse << endl;
        } // End timing
    }
    #pragma endregion

    Transform3D<double> do_3d_alignment(PointCloud<PointIn>::Ptr scene, PointCloud<PointIn>::Ptr object)
    {
        using namespace Eigen;
        pointcloud::show("Before", scene, object);

        PointCloud<PointT>::Ptr object_processed(new PointCloud<PointT>);
        PointCloud<PointT>::Ptr scene_processed(new PointCloud<PointT>);

        preprocess(scene, scene_processed, object, object_processed);

        pointcloud::show("After preprocess", scene_processed, object_processed);


        Matrix4f m = Matrix4f::Identity();
        PointCloud<PointT>::Ptr object_aligned(new PointCloud<PointT>);
        global_align(scene_processed, object_processed, object_aligned, m);

        pointcloud::show("After global estimate", scene_processed, object_aligned);

        local_align(scene_processed, object_aligned, m);

        pointcloud::show("After local estimate", scene_processed, object_aligned);
        
        cout<<"final pose"<<endl<<m<<endl;
        
        Vector3D<> p(m(0,3),m(1,3),m(2,3));
        Rotation3D<> r(
            m(0,0),m(0,1),m(0,2),
            m(1,0),m(1,1),m(1,2),
            m(2,0),m(2,1),m(2,2));
        Transform3D<> t(p,r);
        return t;
    }
}

#endif /*PCL_FUNCTIONS_HPP*/