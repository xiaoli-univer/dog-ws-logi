#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
#include <pcl/filters/voxel_grid.h>

using namespace std;
main (int argc, char **argv)
{
  ros::init (argc, argv, "pcd_viewer_node");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcd_cloud", 1);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudDW(new pcl::PointCloud<pcl::PointXYZI>());


  sensor_msgs::PointCloud2 output;
  string pcd_file_path_ = "", default_val = "./temp.pcd", frame_id_;
  float grid_leaf, freq;
  bool use_downsample = true;
  std::cout << "Over!!" << std::endl;

  ros::param::param<std::string>("~/pcd_file_path", pcd_file_path_, default_val);
  ros::param::param<std::string>("~/pcd_frame_id_", frame_id_, "map");
  ros::param::param<float>("~/downsample_grid", grid_leaf, 0.1);
  ros::param::param<float>("~/frequency", freq, 1.0);
  ros::param::param<bool>("~/use_downsample", use_downsample, true);

	pcl::PCDReader reader;
	reader.read(pcd_file_path_, *cloud);    //读取点云到cloud中
  // pcl::io::loadPCDFile(pcd_file_path_, *cloud);
  std::cout << "Load Over!!" << std::endl;
  std::cout << "Total points: " << cloud->size() << std::endl;

  // DownSampling
  if(use_downsample){
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);            //设置需要过滤的点云给滤波对象
    sor.setLeafSize(grid_leaf, grid_leaf, grid_leaf);  //设置滤波时创建的体素体积为1cm的立方体
    sor.filter(*cloudDW);           //执行滤波处理，存储输出
    std::cout << "DownSampling Over!!" << std::endl;
    std::cout << "Total downsampled points: " << cloudDW->size() << std::endl;
  }
  else{
    std::cout << "Fully Loading!!" << std::endl;
    *cloudDW = *cloud;
  }

  //Convert the cloud to ROS message
  pcl::toROSMsg(*cloudDW, output);
  pcl::io::savePCDFile("/home/jetson/Dog_ws/src/FAST_LIO/PCD/f1_dog.pcd",*cloudDW);
  output.header.frame_id = frame_id_;//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer
  ros::Rate loop_rate(freq);
  while (ros::ok())
  {
    std::cout << "Total points: " << cloudDW->size() << std::endl;
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}