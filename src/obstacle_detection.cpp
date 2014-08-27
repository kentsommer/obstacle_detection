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

#include <iostream>
#include <boost/shared_ptr.hpp>

// PCL specific includes
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
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

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Type Defs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

using namespace std;

ros::Publisher pub_obstacles;
ros::Publisher pub_ground_plane;
ros::Publisher pub_cluster;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  boost::shared_ptr<pcl::PCLPointCloud2> cloud(new pcl::PCLPointCloud2);
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

  boost::shared_ptr<pcl::PCLPointCloud2> temp(new pcl::PCLPointCloud2);
  *temp = *cloud;

  //// Did I mention pointers?
  pcl::PCLPointCloud2ConstPtr tempPtr(temp);
  pcl::PointCloud<pcl::PointXYZ>::Ptr groundcloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr groundcoeffPtr(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr groundindicPtr(new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr groundpointsPtr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr objectpointsPtr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudprojPtr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr groundhullPtr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndices::Ptr objectindicPtr(new pcl::PointIndices);

  //// Find the ground plane using RANSAC
  pcl::fromPCLPointCloud2(*temp, *groundcloudPtr);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (200);
  seg.setDistanceThreshold (0.014); // 0.0195
  seg.setInputCloud (groundcloudPtr);
  seg.segment (*groundindicPtr, *groundcoeffPtr);
  ROS_DEBUG_STREAM("Ground cloud before filtering: "
                   << groundcloudPtr->height * groundcloudPtr->width << " data points.");

  ////Extract ground plane inliers
  pcl::ExtractIndices<pcl::PointXYZ> extractor;
  extractor.setInputCloud(groundcloudPtr);
  extractor.setIndices(groundindicPtr);
  extractor.filter(*groundpointsPtr);
  ROS_DEBUG_STREAM("Ground cloud after filtering: "
                   << groundpointsPtr->height * groundpointsPtr->width << " data points.");

  // publish filtered ground plane
  sensor_msgs::PointCloud2 msg_ground_plane;
  pcl::PCLPointCloud2* ground_plane_cloud = new pcl::PCLPointCloud2;
  pcl::toPCLPointCloud2(*groundpointsPtr, *ground_plane_cloud);
  pcl_conversions::moveFromPCL(*ground_plane_cloud, msg_ground_plane);
  pub_ground_plane.publish(msg_ground_plane);

  ////Extract the ground plane outliers
  pcl::ExtractIndices<pcl::PointXYZ> outlier_extractor;
  outlier_extractor.setInputCloud(groundcloudPtr);
  outlier_extractor.setIndices(groundindicPtr);
  outlier_extractor.setNegative(true);
  outlier_extractor.filter(*objectpointsPtr);
  ROS_DEBUG_STREAM("Objects cloud before filtering: " << objectpointsPtr->height * objectpointsPtr->width
                   << " data points.");

  ////Project the ground inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(groundpointsPtr);
  proj.setModelCoefficients(groundcoeffPtr);
  proj.filter(*cloudprojPtr);

  ////Create Convex Hull of projected inliers
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud(cloudprojPtr);
  chull.reconstruct(*groundhullPtr);

  ////Extract outliers above convex hull
  pcl::ExtractPolygonalPrismData<pcl::PointXYZ> hull_limiter;
  hull_limiter.setInputCloud(objectpointsPtr);
  hull_limiter.setInputPlanarHull(groundhullPtr);
  hull_limiter.setHeightLimits(0, 4); //0 , 4
  hull_limiter.segment(*objectindicPtr);

  pcl::ExtractIndices<pcl::PointXYZ> object_extractor;
  object_extractor.setInputCloud(objectpointsPtr);
  object_extractor.setIndices(objectindicPtr);
  object_extractor.filter(*objectpointsPtr);
  ROS_DEBUG_STREAM("Objects cloud after filtering: " << objectpointsPtr->height * objectpointsPtr->width
                   << " data points.");

  //Convert pcl::CloudT back to PointCloud2 for last filter step
  pcl::PCLPointCloud2* cloud_out = new pcl::PCLPointCloud2();
  pcl::PCLPointCloud2ConstPtr cloudOutPtr(cloud_out);
  pcl::toPCLPointCloud2(*objectpointsPtr, *cloud_out);

  ///////////////////////////////////////////////////
  //                                               //
  //              Filter out the noise v2          //
  //                                               //
  ///////////////////////////////////////////////////
  if ((cloudOutPtr->height * cloudOutPtr->width) > 0)
  {
    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> ror;
    ror.setInputCloud(cloudOutPtr);
    ror.setMinNeighborsInRadius(5);
    ror.setRadiusSearch(0.03);
    ror.filter(*cloud_out);
  }

  ///////////////////////////////////////////////////
  //                                               //
  //          Conversion out and publish           //
  //                                               //
  ///////////////////////////////////////////////////
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(*cloud_out, output);
  pub_obstacles.publish(output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "obstacle_detection");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

  pub_obstacles = nh.advertise<sensor_msgs::PointCloud2> ("filtered_obstacles", 1);
  pub_ground_plane = nh.advertise<sensor_msgs::PointCloud2> ("filtered_ground_plane", 1);

  // Spin
  ros::Rate spin_rate(5);
  while (ros::ok())
  {
    spin_rate.sleep();
    ros::spinOnce();
  }
}
