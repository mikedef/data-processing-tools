// 2020-03
// Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab
// License: MIT

#include <ros/ros.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
//#include <sensor_msg/PointCloud2.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <vector>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH
using namespace pcl;
using namespace std;

// typedef PointXYZRGB PointType;
typedef PointXYZI PointType;




int main(int argc, char** argv)
{
  
  rosbag::Bag bag;
  rosbag::Bag outbag;
  bag.open("/home/mikedef/auvlab/asv/2019-10-08/master.bag", rosbag::bagmode::Read);
  outbag.open("/home/mikedef/auvlab/asv/2019-10-08/rotated_pointcloud.bag", rosbag::bagmode::Write);

  
  std::vector<std::string> topics;
  topics.push_back(string("/velodyne_points"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  foreach (rosbag::MessageInstance const m, view)
    {
      
      const string& topic_name = m.getTopic();
      if (topic_name == "/velodyne_points")
	{
	  // Define a rotation matrix and rotate about the z-axis
	  float theta = -M_PI/2; // Angle of rotation in radians
	  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	  transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
	  
	  cout<<"points"<<endl;
	  PointCloud<PointType>::Ptr transformed_cloud (new PointCloud<PointType>);
	  PointCloud<PointType>::Ptr pc (new PointCloud<PointType>);
	  sensor_msgs::PointCloud2::ConstPtr pointcloud = m.instantiate<sensor_msgs::PointCloud2>();

	  pcl::fromROSMsg(*pointcloud, *pc);
	  //PCLPointCloud2 pc1_pc2;
	  //pcl_conversions::toPCL(pointcloud, pc1_pc2);
	  //fromPCLPointCloud2(pc1_pc2, *pc);

	  transformPointCloud (*pc, *transformed_cloud, transform);

	  //outbag write
	  outbag.write("/rotated_pc", m.getTime(), *transformed_cloud);
	}
      
    }

  bag.close();
  outbag.close();

}        
