#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>



using namespace std;
using namespace pcl;
using namespace pcl::common;
using namespace pcl::io;
using namespace pcl::registration;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;

typedef PointNormal PointT;
typedef Histogram<153> FeatureT;

float plane_threshold = 0.02; 
float x_lower = -0.5; 
float x_upper = 0.2; 
float y_lower = -0.2; 
float y_upper = 0.1;   
float z_lower = -1.2; 
float z_upper = -0.5; 
float iteration1 = 2000;
float iteration2 = 200;


PointCloud<PointT>::Ptr objectT(new PointCloud<PointT>);
PointCloud<PointT>::Ptr sceneT(new PointCloud<PointT>);
PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>);
pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);



	void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq);

	inline float dist_sq(const FeatureT& query, const FeatureT& target) 
	{
		float result = 0.0;
		for(int i = 0; i < FeatureT::descriptorSize(); ++i) {
			const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
			result += diff * diff;
			}
    	return result;
	}

	void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq) {
		idx = 0;
		distsq = dist_sq(query, target[0]);
		for(size_t i = 1; i < target.size(); ++i) {
			const float disti = dist_sq(query, target[i]);
			if(disti < distsq) {
				idx = i;
				distsq = disti;
			}
		}
	}

	
	void voxelGrid( pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud )
	{
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud (input_cloud);
		sor.setLeafSize (0.0085, 0.0085, 0.0085);
		sor.filter (*output_cloud);

	}

	void spatialFilter( pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud )
	{
		pcl::PassThrough<pcl::PointXYZ> filter_x;
		filter_x.setInputCloud(input_cloud);
		filter_x.setFilterFieldName("x");
		filter_x.setFilterLimits(x_lower, x_upper);
		filter_x.filter(*output_cloud);
			
		pcl::PassThrough<pcl::PointXYZ> filter_y;
		filter_y.setInputCloud(input_cloud);
		filter_y.setFilterFieldName("y");
		filter_y.setFilterLimits(y_lower, y_upper);
		filter_y.filter(*output_cloud);

		pcl::PassThrough<pcl::PointXYZ> filter_z;
		filter_z.setInputCloud(input_cloud);
		filter_z.setFilterFieldName("z");
		filter_z.setFilterLimits(z_lower, z_upper);
		filter_z.filter(*output_cloud);
	}

	void planeRemoval( pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud )
	{
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (plane_threshold);
		seg.setInputCloud(input_cloud);
		seg.segment (*inliers, *coefficients);
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(input_cloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*output_cloud);
	}
	
	void globalAligment (float iteration, pcl::PointCloud<PointT>::Ptr object, pcl::PointCloud<PointT>::Ptr scene, pcl::PointCloud<PointT>::Ptr &object_aligned)
	{
	
		{
			PCLVisualizer v("Before global alignment");
			v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
			v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
			v.spin();
		}
	
		// Compute surface normals
		{
			ScopeTime t("Surface normals");
			NormalEstimation<PointT,PointT> ne;
			ne.setKSearch(20);
			
			ne.setInputCloud(object);
			ne.compute(*object);
			
			ne.setInputCloud(scene);
			ne.compute(*scene);
		}
    
		// Compute shape features
		PointCloud<FeatureT>::Ptr object_features(new PointCloud<FeatureT>);
		PointCloud<FeatureT>::Ptr scene_features(new PointCloud<FeatureT>);
		{
			ScopeTime t("Shape features");
			
			SpinImageEstimation<PointT,PointT,FeatureT> spin;
			spin.setRadiusSearch(0.2);
			
			spin.setInputCloud(object);
			spin.setInputNormals(object);
			spin.compute(*object_features);
			
			spin.setInputCloud(scene);
			spin.setInputNormals(scene);
			spin.compute(*scene_features);
		}
    
		// Find feature matches
		Correspondences corr(object_features->size());
		{
			ScopeTime t("Feature matches");
			for(size_t i = 0; i < object_features->size(); ++i) {
				corr[i].index_query = i;
				nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
			}
		}
    
		// Show matches
		{
			PCLVisualizer v("Matches");
			v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
			v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
			v.addCorrespondences<PointT>(object, scene, corr, 1);
			v.spin();
		}
    
		// Create a k-d tree for scene
		search::KdTree<PointNormal> tree;
		tree.setInputCloud(scene);
		
		// Set RANSAC parameters
		const size_t iter = iteration1;
		const float thressq = 0.02 * 0.02;
    
		// Start RANSAC
		Matrix4f pose = Matrix4f::Identity();
		float penalty = FLT_MAX;
		{
			ScopeTime t("RANSAC");
			cout << "Starting RANSAC..." << endl;
			UniformGenerator<int> gen(0, corr.size() - 1);
			for(size_t i = 0; i < iter; ++i) {
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
					if(distsq[j][0] <= thressq)
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
			
			transformPointCloud(*object, *object_aligned, pose);
			
			// Compute inliers and RMSE
			vector<vector<int> > idx;
			vector<vector<float> > distsq;
			tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
			size_t inliers = 0;
			float rmse = 0;
			for(size_t i = 0; i < distsq.size(); ++i)
				if(distsq[i][0] <= thressq)
					++inliers, rmse += distsq[i][0];
			rmse = sqrtf(rmse / inliers);
		
			// Print pose
			cout << "Got the following pose:" << endl << pose << endl;
			cout << "Inliers: " << inliers << "/" << object->size() << endl;
			cout << "RMSE: " << rmse << endl;
			} // End timing
		
			// Show result
			{
			PCLVisualizer v("After global alignment");
			v.addPointCloud<PointT>(object_aligned, PointCloudColorHandlerCustom<PointT>(object_aligned, 0, 255, 0), "object_aligned");
			v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
			v.spin();
		}
	}
	
int main (int argc, char** argv)
{

    if(argc < 2) {
        cout << "Usage: " << argv[0] << "<scene>" << endl;
        return 0;
    }
	
	
	
	pcl::PCDReader readerpcd;
	readerpcd.read (argv[1], *scene);
	readerpcd.read ("../dog.pcd", *object);

	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D (*scene, minPt, maxPt);

	std::cout << "Min x: " << minPt.x << std::endl;
	std::cout << "Max x: " << maxPt.x << std::endl;
	std::cout << "Min y: " << minPt.y << std::endl;
	std::cout << "Max y: " << maxPt.y << std::endl;
	std::cout << "Min z: " << minPt.z << std::endl;
	std::cout << "Max z: " << maxPt.z << std::endl;
 
 
	std::cerr << "PointCloud of scene before filtering: " << scene->width * scene->height 
       << " data points (" << pcl::getFieldsList (*scene) << ").";
	
	voxelGrid ( object, object );
	
	voxelGrid ( scene, scene );

    spatialFilter ( scene, scene ); 

	planeRemoval ( scene, scene );

	
	std::cerr << "PointCloud of scene after filtering: " << scene->width * scene->height 
       << " data points (" << pcl::getFieldsList (*scene) << ").";
	
	pcl::copyPointCloud(*scene, *scene_filtered);

  	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("scene_filtered.pcd", *scene_filtered, false);
	writer.write<pcl::PointXYZ> ("object_filtered.pcd", *object, false);
	
	
	//Global Aligment
	
		pcl::copyPointCloud(*object, *objectT);
		pcl::copyPointCloud(*scene_filtered, *sceneT);
	
		globalAligment (iteration1, objectT, sceneT, object_aligned);
		
		writer.write<PointT> ("object_aligned.pcd", *object_aligned, false);

	//Local Alignment
	{
		PointCloud<PointNormal>::Ptr objectN(new PointCloud<PointNormal>);
		PointCloud<PointNormal>::Ptr sceneN(new PointCloud<PointNormal>);
		loadPCDFile("object_aligned.pcd", *objectN);
		loadPCDFile("scene_filtered.pcd", *sceneN);
		
		// Create a k-d tree for scene
		search::KdTree<PointNormal> tree;
		tree.setInputCloud(sceneN);
		
		// Set ICP parameters
		const size_t iter = iteration2;
		const float thressq = 0.01 * 0.01;
		
		// Start ICP
		Matrix4f pose = Matrix4f::Identity();
		PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>(*objectN));
		{
			ScopeTime t("ICP");
			cout << "Starting ICP..." << endl;
			for(size_t i = 0; i < iter; ++i) {
				// 1) Find closest points
				vector<vector<int> > idx;
				vector<vector<float> > distsq;
				tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
				
				// Threshold and create indices for object/scene and compute RMSE
				vector<int> idxobj;
				vector<int> idxscn;
				for(size_t j = 0; j < idx.size(); ++j) {
					if(distsq[j][0] <= thressq) {
						idxobj.push_back(j);
						idxscn.push_back(idx[j][0]);
					}
				}
				
				// 2) Estimate transformation
				Matrix4f T;
				TransformationEstimationSVD<PointNormal,PointNormal> est;
				est.estimateRigidTransformation(*object_aligned, idxobj, *sceneN, idxscn, T);
				
				// 3) Apply pose
				transformPointCloud(*object_aligned, *object_aligned, T);
				
				// 4) Update result
				pose = T * pose;
			}
			
			// Compute inliers and RMSE
			vector<vector<int> > idx;
			vector<vector<float> > distsq;
			tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
			size_t inliers = 0;
			float rmse = 0;
			for(size_t i = 0; i < distsq.size(); ++i)
				if(distsq[i][0] <= thressq)
					++inliers, rmse += distsq[i][0];
			rmse = sqrtf(rmse / inliers);
		
			// Print pose
			cout << "Got the following pose:" << endl << pose << endl;
			cout << "Inliers: " << inliers << "/" << objectN->size() << endl;
			cout << "RMSE: " << rmse << endl;
		} // End timing
		
		// Show result
		{
			PCLVisualizer v("After local alignment");
			v.addPointCloud<PointNormal>(object_aligned, PointCloudColorHandlerCustom<PointNormal>(object_aligned, 0, 255, 0), "object_aligned");
			v.addPointCloud<PointNormal>(sceneN, PointCloudColorHandlerCustom<PointNormal>(sceneN, 255, 0, 0),"scene");
			v.spin();
		}
	}  
    

	
	
	return (0);
	
}
