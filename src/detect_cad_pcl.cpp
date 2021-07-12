#include "detect_cad_pcl.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::tracking::ParticleXYZRPY ParticleT;
typedef pcl::tracking::ParticleFilterTracker<PointT, ParticleT> ParticleFilter;

// define global ros publishers
ros::Publisher point_cloud_pub_;
ros::Publisher point_cloud_plane_pub_;
ros::Publisher point_cloud_segment_pub_;
ros::Publisher detection_poses_pub_;
ros::Publisher detection_object_pub_;
ros::Publisher detection_marker_pub_;

// define global variables
pcl::PointCloud<PointT>::Ptr in_cad_points_;
visualization_msgs::Marker detection_marker_;
std::string recognized_objects_ns = "recognized_objects";
ParticleFilter::Ptr tracker_;

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

void load_cad_file(std::string filename){
    pcl::PolygonMesh mesh;

    if (pcl::io::loadPolygonFileSTL (filename, mesh) == 0)
    {
        PCL_ERROR("Failed to load STL file\n");
    }

    // Extract cloud from mesh
    pcl::fromPCLPointCloud2(mesh.cloud, *in_cad_points_);
}

void gridSampleApprox (const pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT> &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<PointT> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}

void init_tracker(){
    double downsampling_grid_size =  0.002;

    // Setup partical tracker
    pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointT, ParticleT>::Ptr tracker
        (new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointT, ParticleT> (8));

    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    ParticleT bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.1f;
    bin_size.roll = 0.1f;
    bin_size.pitch = 0.1f;
    bin_size.yaw = 0.1f;

    // TODO Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
    //tracker->setMaximumParticleNum (1000);
    //tracker->setDelta (0.99);
    //tracker->setEpsilon (0.2);
    //tracker->setBinSize (bin_size);

    // TODO Set all parameters for  ParticleFilter
    tracker_ = tracker;
    //tracker_->setTrans (Eigen::Affine3f::Identity ());
    //tracker_->setStepNoiseCovariance (default_step_covariance);
    //tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    //tracker_->setInitialNoiseMean (default_initial_mean);
    //tracker_->setIterationNum (1);
    //tracker_->setParticleNum (600);
    //tracker_->setResampleLikelihoodThr(0.00);
    //tracker_->setUseNormal (false);

    // Setup coherence object for tracking
    pcl::tracking::ApproxNearestPairPointCloudCoherence<PointT>::Ptr coherence
        (new pcl::tracking::ApproxNearestPairPointCloudCoherence<PointT>);

    pcl::tracking::DistanceCoherence<PointT>::Ptr distance_coherence
        (new pcl::tracking::DistanceCoherence<PointT>);
    coherence->addPointCoherence (distance_coherence);

    pcl::search::Octree<PointT>::Ptr search (new pcl::search::Octree<PointT> (0.01));
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);

    //TODO
    //tracker_->setCloudCoherence (coherence);

    //prepare the model of tracker's target
    Eigen::Vector4f c;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
    pcl::PointCloud<PointT>::Ptr transed_ref (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr transed_ref_downsampled (new pcl::PointCloud<PointT>);

    pcl::compute3DCentroid<PointT> (*in_cad_points_, c);
    trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
    pcl::transformPointCloud<PointT> (*in_cad_points_, *transed_ref, trans.inverse());
    gridSampleApprox(transed_ref, *transed_ref_downsampled, downsampling_grid_size);

    //TODO set reference model and trans
    //tracker_->setReferenceCloud (transed_ref_downsampled);
    //tracker_->setTrans (trans);
}

void pointcloud_detect_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);    //Point cloud from ROS msg
    pcl::fromROSMsg(*cloud_msg, *cloud);

    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::PLYWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Create point cloud objects
    pcl::PointCloud<PointT>::Ptr cloud_voxel_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_passthrough_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_inlier_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr seg_coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);
    pcl::PointIndices::Ptr seg_inliers (new pcl::PointIndices);

    // Build a passthrough filter to remove spurious NaNs and scene background
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 3.0);
    pass.filter (*cloud_passthrough_filtered);

    // Perform the actual filtering
    double grid_size = 0.01;
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud_passthrough_filtered);
    sor.setLeafSize (grid_size, grid_size, grid_size);
    sor.filter (*cloud_voxel_filtered);

    if (cloud_voxel_filtered->points.empty ()){
        ROS_ERROR("No points found in grid filtered point cloud");
        return;
    }

    ROS_INFO("Detecting primitive on %lu points: ", cloud_voxel_filtered->points.size());

    if (cloud_voxel_filtered->points.size () > 20000){
        ROS_ERROR("Point cloud too large. Conside increasing voxel grid size.");
        return;
    }

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_voxel_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_voxel_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*plane_inliers, *plane_coefficients);

    if (plane_inliers->indices.size () == 0)
    {
        ROS_ERROR("No plane inliers found");
        return;
    }

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_voxel_filtered);
    extract.setIndices (plane_inliers);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_plane);
    if (cloud_plane->points.empty ()){
        ROS_ERROR("Plane object is empty");
        return;
    } else {
        publish_point_cloud(point_cloud_plane_pub_, cloud_plane, cloud_msg->header.frame_id);
    }

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_inlier_filtered);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (plane_inliers);
    extract_normals.filter (*cloud_normals2);

    // TODO detect CAD object in point cloud
    tracker_->setInputCloud (cloud_inlier_filtered);
	tracker_->compute();
    
    // publish the segmented inliers to topic
    pcl::PointCloud<PointT>::Ptr cloud_detect_object (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_detect_object);
    if (cloud_detect_object->points.empty ()){
        ROS_ERROR("Detected object is empty");
        return;
    } else {
        publish_point_cloud(point_cloud_segment_pub_, cloud_detect_object, cloud_msg->header.frame_id);
    }

    // TODO Extract position and rotation from localisation
    float position[3];
    float rotation[4];

    object_recognition_msgs::RecognizedObject object;
    object.confidence = 1.0; // TODO: get confidence from segmentation
    object.type.key = "i3dr_detected_obj"; // TODO: make this unique

    geometry_msgs::PoseWithCovarianceStamped object_pose;

    object_pose.header.stamp = ros::Time::now();
    object_pose.pose.pose.position.x = position[0];
    object_pose.pose.pose.position.y = position[1];
    object_pose.pose.pose.position.z = position[2];

    // Add rotation to object pose
    tf::Quaternion quat = tf::Quaternion(rotation[0], rotation[1], rotation[2], rotation[3]);
    tf::Quaternion quat_offset;
    quat_offset.setRPY(
        angles::to_degrees(0),
        angles::to_degrees(0),
        angles::to_degrees(90)
    );
    quat *= quat_offset;
    quat.normalize();
    object_pose.pose.pose.orientation.w = quat[0];
    object_pose.pose.pose.orientation.x = quat[1];
    object_pose.pose.pose.orientation.y = quat[2];
    object_pose.pose.pose.orientation.z = quat[3];

    object_pose.header.frame_id = cloud_msg->header.frame_id;

    object.pose = object_pose;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(poseToVector(object_pose.pose.pose));
    transform.setRotation(quat);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), cloud_msg->header.frame_id, object.type.key));

    detection_marker_.header.frame_id = cloud_msg->header.frame_id;
    detection_marker_.header.stamp = ros::Time::now();
    detection_marker_.ns = recognized_objects_ns;
    detection_marker_.id = 0;
    detection_marker_.type = visualization_msgs::Marker::CYLINDER;
    detection_marker_.action = visualization_msgs::Marker::ADD;

    // Set colour to yellow
    detection_marker_.color.r = 1.0f;
    detection_marker_.color.g = 1.0f;
    detection_marker_.color.b = 0.0f;
    detection_marker_.color.a = 1.0;
    
    // Seems to work for mm models
    detection_marker_.scale.x = 0.1;
    detection_marker_.scale.y = 0.1;
    detection_marker_.scale.z = 0.1;
    
    detection_marker_.pose.orientation = object_pose.pose.pose.orientation;
    detection_marker_.pose.position = object_pose.pose.pose.position;

    //detection_marker_.text = detection_object_type_str;

    detection_object_pub_.publish(object);
    detection_poses_pub_.publish(object.pose);
    detection_marker_pub_.publish(detection_marker_);
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
        ROS_INFO("cad_filepath: %s", in_cad_filepath.c_str());
    } else {
        ROS_ERROR("No cad_filepath specified. Exiting.");
        return 1;
    }

    // load CAD from file
    // convert CAD file to point cloud
    load_cad_file(in_cad_filepath);
    init_tracker();

    point_cloud_segment_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points2_segment", 1);
    point_cloud_plane_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points2_plane", 1);

    detection_poses_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 1);
    detection_object_pub_ = nh.advertise<object_recognition_msgs::RecognizedObject> ("recognized_objects", 1);
    detection_marker_pub_ = nh.advertise<visualization_msgs::Marker> ("object_marker", 1);

    ros::Subscriber pcl_detect_sub = nh.subscribe("points2", 1, pointcloud_detect_callback);
    
    ros::spin();

    return 0;
}