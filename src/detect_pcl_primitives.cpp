#include "detect_pcl_primitives.h"

typedef pcl::PointXYZRGB PointT;
// enum for supported detection object types
enum DetectionObjectType {
    DETECTION_OBJECT_UNKNOWN,
    DETECTION_OBJECT_CYLINDER,
    DETECTION_OBJECT_SPHERE
};

// define global ros publishers
ros::Publisher point_cloud_pub_;
ros::Publisher point_cloud_plane_pub_;
ros::Publisher point_cloud_segment_pub_;
ros::Publisher detection_poses_pub_;
ros::Publisher detection_object_pub_;
ros::Publisher detection_marker_pub_;

// define global variables
DetectionObjectType detection_object_type_ = DETECTION_OBJECT_UNKNOWN;
visualization_msgs::Marker detection_marker_;
std::string recognized_objects_ns = "recognized_objects";

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

    // Choose SACMODEL from DetectionObjectType
    pcl::SacModel sac_model;
    std::string detection_object_type_str;
    switch(detection_object_type_){
        case DETECTION_OBJECT_CYLINDER:
            detection_object_type_str = "cylinder";
            sac_model = pcl::SACMODEL_CYLINDER;
            break;
        case DETECTION_OBJECT_SPHERE:
            detection_object_type_str = "sphere";
            sac_model = pcl::SACMODEL_SPHERE;
            break;
        default:
            ROS_ERROR("Invalid detection object type enum: %d. line: %d, file: %s", detection_object_type_, __LINE__, __FILE__);
            return;
    }

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

    // Create the segmentation object for sphere segmentation and set all the parameters
    seg.setOptimizeCoefficients (false);
    seg.setModelType (sac_model);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (5000);
    seg.setDistanceThreshold (0.01);
    seg.setRadiusLimits (0.0, 0.06);
    seg.setInputCloud (cloud_inlier_filtered);
    seg.setInputNormals (cloud_normals2);
    // Obtain the cylinder inliers and coefficients
    seg.segment (*seg_inliers, *seg_coefficients);

    if (seg_coefficients->values[2] < 0){
        ROS_ERROR(
            "Invalid segmentation coefficients: (%.3f,%.3f,%.3f)",
            seg_coefficients->values[0], seg_coefficients->values[1], seg_coefficients->values[2]
        );
        return;
    }

    if (seg_inliers->indices.size () == 0)
    {
        ROS_ERROR("No segmentation inliers found");
        return;
    }

    // publish the segmented inliers to topic
    extract.setInputCloud (cloud_inlier_filtered);
    extract.setIndices (seg_inliers);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.empty ()){
        ROS_ERROR("Segmented object is empty");
        return;
    } else {
        publish_point_cloud(point_cloud_segment_pub_, cloud_cylinder, cloud_msg->header.frame_id);
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cylinder, centroid);


    
    // Coefficents from RANSAC are different for each model type
    // (https://pointclouds.org/documentation/group__sample__consensus.html)
    // SACMODEL_SPHERE:
    //      Used to determine sphere models. 
    //      The four coefficients of the sphere are given by its 3D center and radius as:
    //          [center.x center.y center.z radius]
    // SACMODEL_CYLINDER:
    //      Used to determine cylinder models.
    //      The seven coefficients of the cylinder are given by a point on its axis, the axis direction, and a radius, as:
    //          [point_on_axis.x point_on_axis.y point_on_axis.z axis_direction.x axis_direction.y axis_direction.z radius]

    // Extract position and rotation from coefficients for each model type
    float position[3];
    float rotation[4];

    switch(detection_object_type_){
        case DETECTION_OBJECT_CYLINDER:
            // Extract position and rotation from coefficients for cylinder model
            position[0] = seg_coefficients->values[0];
            position[1] = seg_coefficients->values[1];
            position[2] = seg_coefficients->values[2];
            // position[0] = centroid[0];
            // position[1] = centroid[1];
            // position[2] = centroid[2];
            rotation[0] = seg_coefficients->values[3];
            rotation[1] = seg_coefficients->values[4];
            rotation[2] = seg_coefficients->values[5];
            rotation[3] = seg_coefficients->values[6];
            // rotation[0] = 0;
            // rotation[1] = 0;
            // rotation[2] = 0;
            // rotation[3] = 1.0;
            break;
        case DETECTION_OBJECT_SPHERE:
            // Extract position and rotation from coefficients for sphere model
            // Rotation is irrelevent for a sphere
            position[0] = seg_coefficients->values[0];
            position[1] = seg_coefficients->values[1];
            position[2] = seg_coefficients->values[2];
            // position[0] = centroid[0];
            // position[1] = centroid[1];
            // position[2] = centroid[2];
            rotation[0] = 0;
            rotation[1] = 0;
            rotation[2] = 0;
            rotation[3] = 1.0;
            break;
        default:
            ROS_ERROR("Invalid detection object type enum: %d. line: %d, file: %s", detection_object_type_, __LINE__, __FILE__);
            return;
    }

    ROS_INFO(
        "Object detected at: (%.3f, %.3f, %.3f) (%.3f,%.3f,%.3f,%.3f)",
        position[0], position[1], position[2],
        rotation[0], rotation[1], rotation[2], rotation[3]
    );

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
    ros::init(argc, argv, "detect_pcl_primitives");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    std::string ns = ros::this_node::getNamespace();

    detection_object_type_ = DETECTION_OBJECT_CYLINDER;

    point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points2", 1);
    point_cloud_segment_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points2_segment", 1);
    point_cloud_plane_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points2_plane", 1);

    detection_poses_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 1);
    detection_object_pub_ = nh.advertise<object_recognition_msgs::RecognizedObject> ("recognized_objects", 1);
    detection_marker_pub_ = nh.advertise<visualization_msgs::Marker> ("object_marker", 1);

    ros::Subscriber pcl_detect_sub = nh.subscribe("points2", 1, pointcloud_detect_callback);
    
    ros::spin();

    return 0;
}