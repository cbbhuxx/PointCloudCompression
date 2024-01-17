#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "publisher");
 
    // 创建节点句柄
    ros::NodeHandle n;
    
    // ros::Publisher point_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher point_pub =n.advertise<std_msgs::String>("Compression_PointCloud",1);

    int count = 0;

    pcl::PointCloud<pcl::PointXYZ> cloud;
        /*从 pcd文件  读取 点云*/
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/robot/my_project/PointCloudCompression/src/sample.pcd",cloud)==-1)
    {    
        //  返回值是 -1  代表 没有读到
        PCL_ERROR("Couldn't read file sample.pcd \n");
        return  (-1);
    }
    //打印压缩前从文件中读取的点云
    std::cout<<"before"<<std::endl;
    for(size_t i;i<cloud.points.size();++i)
    { 
        std::cout<<"x="<<cloud.points[i].x<<"\t"<<"y="<<cloud.points[i].y<<"\t"<<"z="<<cloud.points[i].z<<std::endl;
    }
    /* 设置压缩 和解压 时 的结果信息是否打印显示出来  */
    bool showStatistics = true ;
    //bool showStatistics = false;
    pcl::io::compression_Profiles_e compressionProfile = pcl::io::MANUAL_CONFIGURATION;

    while (ros::ok())
    {

        /*声明八叉树的压缩 变量  指针*/
        pcl::io::OctreePointCloudCompression<pcl::PointXYZ> * PointCloudEncoder;
        /*  进行 八叉树 的 压缩 配置  */
        PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(compressionProfile,showStatistics,0.001,0.01,true,100,true,8);//输入参数
        //压缩的字节流
        std::stringstream compressedData;
        /* 实现压缩 的  函数   第一个参数要求时只能指针，  .makeShared()即将cloud转成了 ptr */
        PointCloudEncoder->encodePointCloud(cloud.makeShared(),compressedData);

        std_msgs::String msg;
        // 获取压缩流的原始数据大小
        std::streampos compressed_size = compressedData.tellp();

        // 确保msg.compressed_data有足够的容量来存放压缩数据
        msg.data.resize(compressed_size);

        // 将压缩流中的数据移动到msg.compressed_data中
        compressedData.seekg(0, std::ios_base::beg);
        compressedData.read(reinterpret_cast<char*>(&msg.data[0]), compressed_size);

        // msg.data = compressedData.str();
        point_pub.publish(msg);
        delete (PointCloudEncoder);
    }

 
    return 0;
}