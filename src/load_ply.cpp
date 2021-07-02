#include <load_ply.h>

// Load point cloud from a PLY file
pcl::PCLPointCloud2::Ptr load_point_cloud(std::string filepath,std::string frame_id){
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  // Load point cloud from PLY file
  pcl::PLYReader reader;
  ROS_INFO("Reading point cloud: %s", filepath.c_str());
  reader.read(filepath, *cloud);
  // Set custom frame_id in header
  cloud->header.frame_id = frame_id;
  // Set time to now in header
  pcl_conversions::toPCL(ros::Time::now(),cloud->header.stamp);
  // Display loaded point cloud size
  ROS_INFO("point cloud loaded.");
  int point_cloud_size = cloud->width;
  ROS_INFO("%d",point_cloud_size);
  return(cloud);
}

// ROS node to load point clouds from PLY file and publish to ROS topic
int main(int argc, char **argv)
{
  // ROS node initialization
  ros::init(argc, argv, "load_ply");
  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");

  double publish_rate, grid_size;
  std::string pcl2_output, filepath, frame_id;
  ros::Publisher point_cloud_pub;

  // Get params
  // get filepath param
  if (p_nh.hasParam("filepath")){
    p_nh.getParam("filepath", filepath);
  } else {
    ROS_ERROR("filepath param not found");
  }

  // get frame_id param
  if (p_nh.hasParam("frame_id")){
    p_nh.getParam("frame_id", frame_id);
  } else {
    ROS_ERROR("frame_id param not found");
  }

  // get pub_rate param
  if (p_nh.hasParam("publish_rate")){
    p_nh.getParam("publish_rate", publish_rate);
  } else {
    publish_rate = 1.0;
  }

  // get grid_size param
  if (p_nh.hasParam("grid_size")){
    p_nh.getParam("grid_size", grid_size);
  } else {
    grid_size = 0.0;
  }

  // set ros loop rate (point cloud publish rate)
  ros::Rate loop_rate(publish_rate);

  // advertise point cloud topic
  point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("points2", 1);

  // load point cloud from file
  pcl::PCLPointCloud2::Ptr loaded_pcl2 = load_point_cloud(filepath, frame_id);

  // reduce point cloud with voxel from if grid size is > 0
  if (grid_size > 0) {
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(loaded_pcl2);
    sor.setLeafSize(grid_size, grid_size, grid_size);
    sor.filter(*loaded_pcl2);
  }

  // publish point cloud repeatedly (rate is set by publish_rate)
  while(ros::ok()){
    point_cloud_pub.publish(loaded_pcl2);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
