#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;
typedef pcl::PointXYZRGB PointT;

int planeMaxIterations = 100;
double planeNormalDistanceWeight = 0.1;
double planeDistanceThreshold = 0.03;

int maxIterations = 10000;
double normalDistanceWeight = 0.2;
double distanceThreshold = 0.1;
double minRadiusLimits = 0;
double maxRadiusLimits = 0.5;

int main(int argc, char** argv) {

	// Object for storing the point clouds data
	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
	pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());

	pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) != 0) {
		return -1;
	}

	// Build a passthrough filter to remove spurious NaNs
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	//pass.setFilterLimits (0, 1.5);
	pass.setFilterLimits (0, 10);
	pass.filter (*cloud_filtered);

	// Estimate point normals
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud);
	ne.setKSearch (50);
	ne.compute (*cloud_normals);

	// Create the segmentation object for the planar model and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight (planeNormalDistanceWeight);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (planeMaxIterations);
	seg.setDistanceThreshold (planeDistanceThreshold);
	seg.setInputCloud (cloud);
	seg.setInputNormals (cloud_normals);

	// Obtain the plane inliers and coefficients
	seg.segment (*inliers_plane, *coefficients_plane);

	// Extract the planar inliers from the input cloud
	extract.setInputCloud (cloud);
	extract.setIndices (inliers_plane);
	extract.setNegative (false);
	extract.filter (*cloud_plane);

	// Remove the planar inliers, extract the rest
	extract.setNegative (true);
	extract.filter (*cloud_filtered2);
	extract_normals.setNegative (true);
	extract_normals.setInputCloud (cloud_normals);
	extract_normals.setIndices (inliers_plane);
	extract_normals.filter (*cloud_normals2);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_CYLINDER);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight (normalDistanceWeight);
	seg.setMaxIterations (maxIterations);
	seg.setDistanceThreshold (distanceThreshold); //0.05
	seg.setRadiusLimits (minRadiusLimits, maxRadiusLimits);
	seg.setInputCloud (cloud_filtered2);
	seg.setInputNormals (cloud_normals2);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_CYLINDER);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight (normalDistanceWeight);
	seg.setMaxIterations (maxIterations);
	seg.setDistanceThreshold (distanceThreshold); //0.05
	seg.setRadiusLimits (minRadiusLimits, maxRadiusLimits);
	seg.setInputCloud (cloud_filtered2);
	seg.setInputNormals (cloud_normals2);

	// Obtain the cylinder inliers and coefficients
	seg.segment (*inliers_cylinder, *coefficients_cylinder);

	extract.setInputCloud (cloud_filtered2);
	extract.setIndices (inliers_cylinder);
	extract.setNegative (false);
	extract.filter (*cloud_cylinder);

	// Visualize it
	pcl::visualization::PCLVisualizer viewer(argv[1]);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandlerCylinder(cloud_cylinder, 34, 139, 34);
	viewer.addPointCloud(cloud_cylinder, colorHandlerCylinder, "Cylinder");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandlerPlane(cloud_plane,128, 128, 128);
	viewer.addPointCloud(cloud_plane, colorHandlerPlane, "Plane");

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
}
