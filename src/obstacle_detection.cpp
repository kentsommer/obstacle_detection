/*
{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}
{} The MIT License (MIT)                                                        {} 
{}                                                                              {}
{} Copyright (c) <2014> <Kent Sommer>                                           {}
{} Permission is hereby granted, free of charge, to any person obtaining a copy {} 
{} of this software and associated documentation files (the "Software"), to deal{}
{} in the Software without restriction, including without limitation the rights {}
{} to use, copy, modify, merge, publish, distribute, sublicense, and/or sell    {} 
{} copies of the Software, and to permit persons to whom the Software is        {}
{} furnished to do so, subject to the following conditions:                     {}
{}                                                                              {} 
{} The above copyright notice and this permission notice shall be included in   {}
{} all copies or substantial portions of the Software.                          {}
{}                                                                              {} 
{} THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR   {}
{} IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,     {}
{} FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE  {} 
{} AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER       {}
{} LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,{}
{} OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN    {}
{} THE SOFTWARE.                                                                {}
{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}
*/

#include <ros/ros.h>
#include <iostream>
// PCL specific includes
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/features/normal_3d.h>

//Filters and Downsampling 
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/radius_outlier_removal.h>

//Clustering 
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>

// Type Defs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

using namespace std;

ros::Publisher pub;
ros::Publisher pub_cluster;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered_out;

  ///////////////////////////////////////////////////
  //                                               //
  //                 Conversion                    //
  //                                               //                            
  ///////////////////////////////////////////////////
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  ///////////////////////////////////////////////////
  //                                               //
  //             Passthrough filter                //
  //                                               //                            
  ///////////////////////////////////////////////////
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud(cloudPtr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.5, 4); //1.3 works well
  pass.filter(*cloud);

  ///////////////////////////////////////////////////
  //                                               //
  //             Voxel Grid Downsample             //
  //                                               //                            
  ///////////////////////////////////////////////////
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.028, 0.028, 0.028);
  sor.filter (*cloud);

  ///////////////////////////////////////////////////
  //                                               //
  //              Filter out the noise             //
  //                                               //                            
  ///////////////////////////////////////////////////
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor2;
  sor2.setInputCloud(cloudPtr);
  sor2.setMeanK(80);
  sor2.setStddevMulThresh(0.15);
  sor2.filter(*cloud);

  ///////////////////////////////////////////////////
  //                                               //
  //             Ground Extraction                 //
  //                                               //                            
  ///////////////////////////////////////////////////

  //// I love pointers, do you love pointers? Someone please make this pretty eventually... it makes me cry
  pcl::PCLPointCloud2* temp = new pcl::PCLPointCloud2;
  *temp = *cloud; 

  //// Did I mention pointers?
  pcl::PCLPointCloud2ConstPtr tempPtr(temp);
  pcl::PointCloud<pcl::PointXYZ>* ground_cloud = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr groundcloudPtr(ground_cloud);

  pcl::ModelCoefficients* ground_coefficients = new pcl::ModelCoefficients;
  pcl::ModelCoefficients::Ptr groundcoeffPtr(ground_coefficients);
  pcl::PointIndices* ground_indices = new pcl::PointIndices; 
  pcl::PointIndices::Ptr groundindicPtr(ground_indices);

  pcl::PointCloud<pcl::PointXYZ>* ground_points = new pcl::PointCloud<pcl::PointXYZ>; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr groundpointsPtr(ground_points);

  pcl::PointCloud<pcl::PointXYZ>* object_points = new pcl::PointCloud<pcl::PointXYZ>; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr objectpointsPtr(object_points);

  pcl::PointCloud<pcl::PointXYZ>* cloud_projected = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudprojPtr(cloud_projected);

  pcl::PointCloud<pcl::PointXYZ>* ground_hull = new pcl::PointCloud<pcl::PointXYZ>; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr groundhullPtr(ground_hull);

  pcl::PointIndices* object_indices = new pcl::PointIndices; 
  pcl::PointIndices::Ptr objectindicPtr(object_indices);

  //// Find the ground plane using RANSAC
  pcl::fromPCLPointCloud2(*temp, *ground_cloud);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (200);
  seg.setDistanceThreshold (0.014); // 0.0195
  seg.setInputCloud (groundcloudPtr);
  seg.segment (*ground_indices, *ground_coefficients);
  ROS_INFO("Ground cloud before filtering: %d data points.", ground_cloud->height * ground_cloud->width); // Debug Print

  ////Extract ground plane inliers
  pcl::ExtractIndices<pcl::PointXYZ> extractor;
  extractor.setInputCloud(groundcloudPtr);
  extractor.setIndices(groundindicPtr);
  extractor.filter(*ground_points);

  ////Extract the ground plane outliers
  pcl::ExtractIndices<pcl::PointXYZ> outlier_extractor;
  outlier_extractor.setInputCloud(groundcloudPtr);
  outlier_extractor.setIndices(groundindicPtr);
  outlier_extractor.setNegative(true);
  outlier_extractor.filter(*object_points);
  ROS_INFO("Objects cloud before filtering: %d data points.", object_points->height * object_points->width);

  ////Project the ground inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(groundpointsPtr);
  proj.setModelCoefficients(groundcoeffPtr);
  proj.filter(*cloud_projected);

  ////Create Convex Hull of projected inliers
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud(cloudprojPtr);
  chull.reconstruct(*ground_hull);

  ////Extract outliers above convex hull
  pcl::ExtractPolygonalPrismData<pcl::PointXYZ> hull_limiter;
  hull_limiter.setInputCloud(objectpointsPtr);
  hull_limiter.setInputPlanarHull(groundhullPtr);
  hull_limiter.setHeightLimits(0, 4); //0 , 4
  hull_limiter.segment(*object_indices);

  pcl::ExtractIndices<pcl::PointXYZ> object_extractor;
  object_extractor.setInputCloud(objectpointsPtr);
  object_extractor.setIndices(objectindicPtr);
  object_extractor.filter(*object_points);
  ROS_INFO("Objects cloud after filtering: %d data points.", object_points->height * object_points->width);

  //Convert pcl::CloudT back to PointCloud2 for last filter step
  pcl::PCLPointCloud2* cloud_out = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudOutPtr(cloud_out);
  pcl::toPCLPointCloud2(*object_points, *cloud_out); // *object_points

  ///////////////////////////////////////////////////
  //                                               //
  //              Filter out the noise v2          //
  //                                               //                            
  ///////////////////////////////////////////////////
  pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> ror;
  ror.setInputCloud(cloudOutPtr);
  ror.setMinNeighborsInRadius(5);
  ror.setRadiusSearch(0.03);
  ror.filter(*cloud_out);


  ///////////////////////////////////////////////////
  //                                               //
  //          Conversion out and publish           //
  //                                               //                            
  ///////////////////////////////////////////////////
  sensor_msgs::PointCloud2 output;
  // pcl_conversions::moveFromPCL(cloud_filtered_out, output);
  pcl_conversions::moveFromPCL(*cloud_out, output);
    // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "obstacle_detection");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 10, cloud_cb);

  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 10);

  // Spin
  ros::spin ();
}
