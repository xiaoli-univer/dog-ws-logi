#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions//pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/filters/passthrough.h>                 //直通滤波器头文件
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>

std::string file_directory;
double thre_z_min = 0.3;
double thre_z_max = 2.0;
int flag_pass_through = 0;
double map_resolution = 0.05;
double thre_radius = 0.1;
//直通滤波后数据指针
pcl::PointCloud<pcl::PointXYZI>::Ptr
    cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZI>);

//直通滤波
void PassThroughFilter(const double &thre_low, const double &thre_high,
                       const bool &flag_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in);

//转换为栅格地图数据并发布
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    nav_msgs::OccupancyGrid &msg);

int main(int argc, char **argv)
{
    ros::init(argc,argv,"UanBdet");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output",1);
    
    // pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::PointCloud2 output;
    pcl::io::loadPCDFile("/home/jetson/Dog_ws/src/FAST_LIO/PCD/scans.pcd", *cloud); 
    //读取文件绝对路径.
    //对数据进行直通滤波
    PassThroughFilter(thre_z_min, thre_z_max, bool(flag_pass_through), cloud);
    
    pcl::toROSMsg(*cloud_after_PassThrough,output);
    
    output.header.frame_id = "map";   
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

//直通滤波器对点云进行过滤，获取设定高度范围内的数据
void PassThroughFilter(const double &thre_low, const double &thre_high,
                       const bool &flag_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in){
  // 创建滤波器对象
  pcl::PassThrough<pcl::PointXYZI> passthrough;
  //输入点云
  passthrough.setInputCloud(cloud_in);
  //设置对z轴进行操作
  passthrough.setFilterFieldName("z");
  //设置滤波范围
  passthrough.setFilterLimits(thre_low, thre_high);
  // true表示保留滤波范围外，false表示保留范围内
  passthrough.setFilterLimitsNegative(flag_in);
  //执行滤波并存储
  passthrough.filter(*cloud_after_PassThrough);
  // test 保存滤波后的点云到文件
  std::cout << "直通滤波后点云数据点数："
            << cloud_after_PassThrough->points.size() << std::endl;
}


