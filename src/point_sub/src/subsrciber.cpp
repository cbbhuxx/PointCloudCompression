#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include "std_msgs/String.h"
 
 

pcl::io::OctreePointCloudCompression<pcl::PointXYZ> * PointCloudDecoder;
bool showStatistics = true ;
pcl::io::compression_Profiles_e compressionProfile = pcl::io::MANUAL_CONFIGURATION;
 
void callback(const std_msgs::String::ConstPtr& msg)//指针
{
    std::stringstream compressedData;
    compressedData.write(msg->data.c_str(), msg->data.size());
    
    //声明 解压 缩 后的 点云 指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>());
    PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ> ();
    PointCloudDecoder->decodePointCloud(compressedData,cloudOut);
    
    //打印解压缩后的点云坐标
    std::cout<<"after"<<std::endl;
    for(size_t i;i<cloudOut->points.size();++i)
    {
        std::cout<<"x="<<cloudOut->points[i].x<<"\t"<<"y="<<cloudOut->points[i].y<<"\t"<<"z="<<cloudOut->points[i].z<<std::endl;
    }

    pcl::io::savePCDFile("output.pcd",*cloudOut);
    
    delete (PointCloudDecoder);
}
 
int main(int argc,char **argv)
{
 
	//初始化ROS节点
	ros::init(argc,argv,"subscriber");
 
	//创建节点语柄
	ros::NodeHandle n;
 
	// 创建一个Subscriber，订阅名为/turtle/pose的topic,注册回调函数poseCallback
	ros::Subscriber pose_sub=n.subscribe("Compression_PointCloud",11,callback);
 
	// 循环等待回调函数
	ros::spin();
 
	return 0;
 
}
