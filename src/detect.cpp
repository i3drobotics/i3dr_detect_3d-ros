#include "detect.h"

using namespace HalconCpp;
using namespace HDevEngineCpp;

struct halcon_variable
{
    std::string name;
    std::vector<std::string> data;
};

ros::Publisher point_cloud_pub_;

// read PLY point cloud from file using PCL
pcl::PointCloud<pcl::PointXYZ>::Ptr read_pcl_from_file(std::string point_cloud_filepath){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile(point_cloud_filepath, *cloud);
    return cloud;
}

void publish_sample_point_cloud(std::string sample_point_cloud_filepath){
    pcl::PointCloud<pcl::PointXYZ>::Ptr sample_pcl = read_pcl_from_file(sample_point_cloud_filepath);
    
    sensor_msgs::PointCloud2 pcl2_msg;
    pcl::toROSMsg(*sample_pcl, pcl2_msg);

    //pcl2_msg.header = header;
    pcl2_msg.height = 1;
    pcl2_msg.width = sample_pcl->size();
    pcl2_msg.is_bigendian = false;
    pcl2_msg.is_dense = false; // there may be invalid points

    //Publish ROS msg
    point_cloud_pub_.publish(pcl2_msg);
    ROS_INFO("Published sample point cloud");
}

void pointcloud_detect_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    using namespace halcon_bridge;
    using namespace HalconCpp;

    ROS_INFO("Converting point cloud to Halcon.");

    auto halcon_ptr = toHalconCopy(cloud_msg);
    HObjectModel3D scene = *halcon_ptr->model;

    HObjectModel3D plane_fit;
    try{
        plane_fit = scene.FitPrimitivesObjectModel3d("primitive_type", "plane");
    } catch (HOperatorException &e){
        ROS_ERROR("Halcon error: %s, line: %d, file: %s", e.ErrorMessage().Text(), __LINE__, __FILE__);
        return;
    }

    HTuple plane_pose_tuple;
    try{
        plane_pose_tuple = plane_fit.GetObjectModel3dParams("primitive_parameter_pose");
    } catch (HOperatorException &e){
        ROS_ERROR("Halcon error: %s, line: %d, file: %s", e.ErrorMessage().Text(), __LINE__, __FILE__);
        return;
    }

    HPose plane_pose;
    HObjectModel3D plane_object;
    try{
        plane_pose = HPose(plane_pose_tuple);
        plane_object.GenPlaneObjectModel3d(plane_pose, HTuple(), HTuple());
    } catch (HOperatorException &e){
        ROS_ERROR("Halcon error: %s, line: %d, file: %s", e.ErrorMessage().Text(), __LINE__, __FILE__);
        return;
    }

    HPose empty_pose;
    HTuple plane_distance;
    try{
        double max_plane_distance = 0.0;
        scene.DistanceObjectModel3d(plane_object, HPose(), max_plane_distance, "distance_to", "primitive");
        plane_distance = scene.GetObjectModel3dParams("&distance");
    } catch (HOperatorException &e){
        ROS_ERROR("Halcon error: %s, line: %d, file: %s", e.ErrorMessage().Text(), __LINE__, __FILE__);
        return;
    }

    HTuple plane_min;
    HTuple plane_max;
    try{
        plane_min = plane_distance.TupleMin();
        plane_max = plane_distance.TupleMax();
    } catch (HOperatorException &e){
        ROS_ERROR("Halcon error: %s, line: %d, file: %s", e.ErrorMessage().Text(), __LINE__, __FILE__);
        return;
    }

    HObjectModel3D scene_thresholded;
    try{
        scene_thresholded = scene.SelectPointsObjectModel3d("&distance", plane_min, plane_max);
    } catch (HOperatorException &e){
        ROS_ERROR("Halcon error: %s, line: %d, file: %s", e.ErrorMessage().Text(), __LINE__, __FILE__);
        return;
    }

    HObjectModel3DArray connected_objects;
    try{
        connected_objects = scene_thresholded.ConnectionObjectModel3d("distance_3d", 0.01);
    } catch (HOperatorException &e){
        ROS_ERROR("Halcon error: %s, line: %d, file: %s", e.ErrorMessage().Text(), __LINE__, __FILE__);
        return;
    }

    HObjectModel3DArray selected_objects;
    try{
        selected_objects = scene.SelectObjectModel3d(connected_objects, "num_points", "and", 1000, 500000);
    } catch (HOperatorException &e){
        ROS_ERROR("Halcon error: %s, line: %d, file: %s", e.ErrorMessage().Text(), __LINE__, __FILE__);
        return;
    }

    try{
        for(int i = 0; i < selected_objects.Length(); i++){
            HObjectModel3D primitive_fit = selected_objects.Tools()[i].FitPrimitivesObjectModel3d("primitive_type", "all");
            HTuple primitive_pose = primitive_fit.GetObjectModel3dParams("center");
            double x = primitive_pose[0];
            double y = primitive_pose[1];
            double z = primitive_pose[2];
            std::cout << x << ", " << y << ", " << z <<  std::endl;
        }
    } catch (HOperatorException &e){
        ROS_ERROR("Halcon error: %s, line: %d, file: %s", e.ErrorMessage().Text(), __LINE__, __FILE__);
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    std::string ns = ros::this_node::getNamespace();

    std::string point_cloud_filepath = "data/point_clouds/ball001.ply";

    point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points2", 1);

    ros::Subscriber sub = nh.subscribe("points2", 1, pointcloud_detect_callback);
    
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        publish_sample_point_cloud(point_cloud_filepath);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}