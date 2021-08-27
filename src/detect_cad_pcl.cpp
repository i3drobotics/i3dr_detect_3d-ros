#include "detect_cad_pcl.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::Normal NormalT;
typedef pcl::ReferenceFrame RefFrameT;
typedef pcl::SHOT352 DescriptorT;

// define global ros publishers
ros::Publisher point_cloud_pub_;
ros::Publisher point_cloud_plane_pub_;
ros::Publisher point_cloud_plane_removed_pub_;
ros::Publisher point_cloud_segment_pub_;
ros::Publisher point_cloud_in_cad_pub_;
ros::Publisher point_cloud_cad_ref_pub_;
ros::Publisher detection_poses_pub_;
ros::Publisher detection_object_pub_;
ros::Publisher detection_marker_pub_;

// define global variables
std::string local_frame_id_ = "world"; //will update this based on the frame_id of the point cloud input
pcl::PointCloud<PointT>::Ptr cad_model_points_;
visualization_msgs::Marker detection_marker_;
std::string recognized_objects_ns = "recognized_objects";
// pcl::IterativeClosestPoint<PointT, PointT> icp_;

double computeCloudResolution (const pcl::PointCloud<PointT>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointT> tree;
  tree.setInputCloud (cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

tf::Vector3 poseToVector(const geometry_msgs::Pose& pose) {
	const float px = static_cast<float>(pose.position.x);
	const float py = static_cast<float>(pose.position.y);
	const float pz = static_cast<float>(pose.position.z);
	return tf::Vector3({px, py, pz});
}

// Convert ROS pose to Eigen::Quaternionf
tf::Quaternion poseToQuaternion(const geometry_msgs::Pose& pose) {
	const float w = static_cast<float>(pose.orientation.w);
	const float x = static_cast<float>(pose.orientation.x);
	const float y = static_cast<float>(pose.orientation.y);
	const float z = static_cast<float>(pose.orientation.z);
	return tf::Quaternion(w, x, y, z);
}

void publish_point_cloud(ros::Publisher pub, pcl::PointCloud<PointT>::Ptr cloud, std::string frame_id){
    sensor_msgs::PointCloud2 pcl2_msg;
    pcl::toROSMsg(*cloud, pcl2_msg);
    pcl2_msg.header.frame_id = frame_id;
    pub.publish(pcl2_msg);
}

void load_stl_file(std::string filepath){
    pcl::PolygonMesh mesh;
    cad_model_points_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());

    // load mesh from stl file
    if (pcl::io::loadPolygonFileSTL(filepath, mesh) == 0)
    {
        PCL_ERROR("Failed to load STL file\n");
    }

    // Extract cloud from mesh
    pcl::fromPCLPointCloud2(mesh.cloud, *cad_model_points_);
}

void load_ply_file(std::string filepath){
    cad_model_points_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());

    // Load point cloud from PLY file
    pcl::PLYReader reader;
    ROS_INFO("Reading point cloud: %s", filepath.c_str());
    reader.read(filepath, *cad_model_points_);
    // Set time to now in headers
    pcl_conversions::toPCL(ros::Time::now(), cad_model_points_->header.stamp);
    // Display loaded point cloud size
    ROS_INFO("point cloud loaded.");
    int point_cloud_size = cad_model_points_->width;
    ROS_INFO("%d",point_cloud_size);
}

void load_pcd_file(std::string filepath){
    cad_model_points_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());

    // Load point cloud from PCD file
    pcl::PCDReader reader;
    ROS_INFO("Reading point cloud: %s", filepath.c_str());
    reader.read(filepath, *cad_model_points_);
    // Set time to now in headers
    pcl_conversions::toPCL(ros::Time::now(), cad_model_points_->header.stamp);
    // Display loaded point cloud size
    ROS_INFO("point cloud loaded.");
    int point_cloud_size = cad_model_points_->width;
    ROS_INFO("%d",point_cloud_size);
}

void gridSampleApprox (const pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT> &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<PointT> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}

void init_detection(){
    
}

void pointcloud_detect_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    local_frame_id_ = cloud_msg->header.frame_id;
    ROS_INFO("FrameID: %s", local_frame_id_.c_str());

    // Use correspondence grouping in PCL to detect location of model in scene
    // https://pcl.readthedocs.io/projects/tutorials/en/latest/correspondence_grouping.html#correspondence-grouping

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);    //Point cloud from ROS msg
    pcl::fromROSMsg(*cloud_msg, *cloud);

    float model_ss_ (0.01f);
    float scene_ss_ (0.03f);
    float rf_rad_ (0.015f);
    float descr_rad_ (0.02f);
    float cg_size_ (0.01f);
    float cg_thresh_ (5.0f);
    bool use_hough_ = true;

    pcl::PassThrough<PointT> pass;
    pcl::PointCloud<PointT>::Ptr cloud_passthrough_filtered (new pcl::PointCloud<PointT>);

    pcl::PointCloud<PointT>::Ptr model (new pcl::PointCloud<PointT> ());
    pcl::PointCloud<PointT>::Ptr model_keypoints (new pcl::PointCloud<PointT> ());
    pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT> ());
    pcl::PointCloud<PointT>::Ptr scene_keypoints (new pcl::PointCloud<PointT> ());
    pcl::PointCloud<NormalT>::Ptr model_normals (new pcl::PointCloud<NormalT> ());
    pcl::PointCloud<NormalT>::Ptr scene_normals (new pcl::PointCloud<NormalT> ());
    pcl::PointCloud<DescriptorT>::Ptr model_descriptors (new pcl::PointCloud<DescriptorT> ());
    pcl::PointCloud<DescriptorT>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorT> ());


    // Build a passthrough filter to remove spurious NaNs and scene background
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 2.0);
    pass.filter (*cloud_passthrough_filtered);

    ROS_INFO("Detecting object in %lu points: ", cloud_passthrough_filtered->points.size());

    // get model from input cad
    pcl::copyPointCloud(*cad_model_points_, *model);
    // get scene from camera 3D
    pcl::copyPointCloud(*cloud_passthrough_filtered, *scene);

    // normalise points
    pcl::NormalEstimationOMP<PointT, NormalT> norm_est;
    norm_est.setKSearch (10);
    norm_est.setInputCloud (model);
    norm_est.compute (*model_normals);
    norm_est.setInputCloud (scene);
    norm_est.compute (*scene_normals);

    // downsample clouds to extract keypoints
    pcl::UniformSampling<PointT> uniform_sampling;
    uniform_sampling.setInputCloud (model);
    uniform_sampling.setRadiusSearch (model_ss_);
    uniform_sampling.filter (*model_keypoints);
    std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

    uniform_sampling.setInputCloud (scene);
    uniform_sampling.setRadiusSearch (scene_ss_);
    uniform_sampling.filter (*scene_keypoints);
    std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

    // compute descriptor for keypoints
    pcl::SHOTEstimationOMP<PointT, NormalT, DescriptorT> descr_est;
    descr_est.setRadiusSearch (descr_rad_);

    descr_est.setInputCloud (model_keypoints);
    descr_est.setInputNormals (model_normals);
    descr_est.setSearchSurface (model);
    descr_est.compute (*model_descriptors);

    descr_est.setInputCloud (scene_keypoints);
    descr_est.setInputNormals (scene_normals);
    descr_est.setSearchSurface (scene);
    descr_est.compute (*scene_descriptors);

    // find model-scene correspondences with KdTree
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    pcl::KdTreeFLANN<DescriptorT> match_search;
    match_search.setInputCloud (model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
    {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);
        if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
        {
        continue;
        }
        int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
        if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        model_scene_corrs->push_back (corr);
        }
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

    // actual clustering
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    //  using Hough3D
    if (use_hough_)
    {
        // compute (Keypoints) Reference Frames only for Hough
        pcl::PointCloud<RefFrameT>::Ptr model_rf (new pcl::PointCloud<RefFrameT> ());
        pcl::PointCloud<RefFrameT>::Ptr scene_rf (new pcl::PointCloud<RefFrameT> ());

        pcl::BOARDLocalReferenceFrameEstimation<PointT, NormalT, RefFrameT> rf_est;
        rf_est.setFindHoles (true);
        rf_est.setRadiusSearch (rf_rad_);

        rf_est.setInputCloud (model_keypoints);
        rf_est.setInputNormals (model_normals);
        rf_est.setSearchSurface (model);
        rf_est.compute (*model_rf);

        rf_est.setInputCloud (scene_keypoints);
        rf_est.setInputNormals (scene_normals);
        rf_est.setSearchSurface (scene);
        rf_est.compute (*scene_rf);

        //  Clustering
        pcl::Hough3DGrouping<PointT, PointT, RefFrameT, RefFrameT> clusterer;
        clusterer.setHoughBinSize (cg_size_);
        clusterer.setHoughThreshold (cg_thresh_);
        clusterer.setUseInterpolation (true);
        clusterer.setUseDistanceWeight (false);

        clusterer.setInputCloud (model_keypoints);
        clusterer.setInputRf (model_rf);
        clusterer.setSceneCloud (scene_keypoints);
        clusterer.setSceneRf (scene_rf);
        clusterer.setModelSceneCorrespondences (model_scene_corrs);

        //clusterer.cluster (clustered_corrs);
        clusterer.recognize (rototranslations, clustered_corrs);
    }
    else // Using GeometricConsistency
    {
        pcl::GeometricConsistencyGrouping<PointT, PointT> gc_clusterer;
        gc_clusterer.setGCSize (cg_size_);
        gc_clusterer.setGCThreshold (cg_thresh_);

        gc_clusterer.setInputCloud (model_keypoints);
        gc_clusterer.setSceneCloud (scene_keypoints);
        gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

        //gc_clusterer.cluster (clustered_corrs);
        gc_clusterer.recognize (rototranslations, clustered_corrs);
    }

    // output results
    std::cout << "Model instances found: " << rototranslations.size () << std::endl;
    for (std::size_t i = 0; i < rototranslations.size (); ++i)
    {
        std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
        Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

        printf ("\n");
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
        printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
        printf ("\n");
        printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
    }



   //publish_point_cloud(point_cloud_segment_pub_, icp_result, cloud_msg->header.frame_id);
    
    
    
    
    // // TODO Extract position and rotation from localisation
    // float position[3];
    // float rotation[4];

    // object_recognition_msgs::RecognizedObject object;
    // object.confidence = 1.0; // TODO: get confidence from segmentation
    // object.type.key = "i3dr_detected_obj"; // TODO: make this unique

    // geometry_msgs::PoseWithCovarianceStamped object_pose;

    // object_pose.header.stamp = ros::Time::now();
    // object_pose.pose.pose.position.x = position[0];
    // object_pose.pose.pose.position.y = position[1];
    // object_pose.pose.pose.position.z = position[2];

    // // Add rotation to object pose
    // tf::Quaternion quat = tf::Quaternion(rotation[0], rotation[1], rotation[2], rotation[3]);
    // tf::Quaternion quat_offset;
    // quat_offset.setRPY(
    //     angles::to_degrees(0),
    //     angles::to_degrees(0),
    //     angles::to_degrees(90)
    // );
    // quat *= quat_offset;
    // quat.normalize();
    // object_pose.pose.pose.orientation.w = quat[0];
    // object_pose.pose.pose.orientation.x = quat[1];
    // object_pose.pose.pose.orientation.y = quat[2];
    // object_pose.pose.pose.orientation.z = quat[3];

    // object_pose.header.frame_id = cloud_msg->header.frame_id;

    // object.pose = object_pose;

    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // transform.setOrigin(poseToVector(object_pose.pose.pose));
    // transform.setRotation(quat);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), cloud_msg->header.frame_id, object.type.key));

    // detection_marker_.header.frame_id = cloud_msg->header.frame_id;
    // detection_marker_.header.stamp = ros::Time::now();
    // detection_marker_.ns = recognized_objects_ns;
    // detection_marker_.id = 0;
    // detection_marker_.type = visualization_msgs::Marker::CYLINDER;
    // detection_marker_.action = visualization_msgs::Marker::ADD;

    // // Set colour to yellow
    // detection_marker_.color.r = 1.0f;
    // detection_marker_.color.g = 1.0f;
    // detection_marker_.color.b = 0.0f;
    // detection_marker_.color.a = 1.0;
    
    // // Seems to work for mm models
    // detection_marker_.scale.x = 0.1;
    // detection_marker_.scale.y = 0.1;
    // detection_marker_.scale.z = 0.1;
    
    // detection_marker_.pose.orientation = object_pose.pose.pose.orientation;
    // detection_marker_.pose.position = object_pose.pose.pose.position;

    // //detection_marker_.text = detection_object_type_str;

    // detection_object_pub_.publish(object);
    // detection_poses_pub_.publish(object.pose);
    // detection_marker_pub_.publish(detection_marker_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_cad");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    std::string ns = ros::this_node::getNamespace();

    std::string in_cad_filepath;

    if (p_nh.getParam("cad_filepath", in_cad_filepath))
    {
        ROS_INFO("cad_filepath:ply%s", in_cad_filepath.c_str());
    } else {
        ROS_ERROR("No cad_filepath specified. Exiting.");
        return 1;
    }

    std::string cad_file_extension = in_cad_filepath.substr(in_cad_filepath.find_last_of(".") + 1);

    if(cad_file_extension == "stl") {
        load_stl_file(in_cad_filepath);
    }
    else if(cad_file_extension == "ply") {
        load_ply_file(in_cad_filepath);
    }
    else if(cad_file_extension == "pcd") {
        load_pcd_file(in_cad_filepath);
    }
    else {
        ROS_ERROR("CAD file type %s not supported. Exiting.",cad_file_extension.c_str());
        return 1;
    }

    if (cad_model_points_->points.empty ()){
        ROS_ERROR("No points found in cad point cloud");
        return 1;
    }

    // transform loaded model to world zero
    Eigen::Vector4f c;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
    pcl::PointCloud<PointT>::Ptr transed_ref (new pcl::PointCloud<PointT>);

    pcl::compute3DCentroid<PointT> (*cad_model_points_, c);
    trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
    ROS_INFO("Input CAD centroid (%.3f,%.3f,%.3f)", c[0], c[1], c[2]);
    //pcl::transformPointCloud<PointT> (*cad_model_points_, *transed_ref, trans.inverse());

    init_detection();

    point_cloud_segment_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points2_segment", 1);
    point_cloud_plane_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points2_plane", 1);
    point_cloud_plane_removed_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points2_plane_removed", 1);
    point_cloud_cad_ref_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points2_cad_ref", 1);

    detection_poses_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 1);
    detection_object_pub_ = nh.advertise<object_recognition_msgs::RecognizedObject> ("recognized_objects", 1);
    detection_marker_pub_ = nh.advertise<visualization_msgs::Marker> ("object_marker", 1);

    ros::Subscriber pcl_detect_sub = nh.subscribe("points2", 1, pointcloud_detect_callback);

    //TODO set this with parameter
    ros::Rate loop_rate(1);

    while(ros::ok()){
        publish_point_cloud(point_cloud_cad_ref_pub_, cad_model_points_, local_frame_id_);
        ros::spinOnce();
        loop_rate.sleep(); 
    }

    return 0;
}