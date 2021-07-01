#include <ros/ros.h>
#include <ros/console.h>
#include <ros/param.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <boost/filesystem.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <halconcpp/HalconCpp.h>
#include <hdevengine/HDevEngineCpp.h>

using namespace HalconCpp;
using namespace HDevEngineCpp;

struct halcon_variable
{
    std::string name;
    std::vector<std::string> data;
};

std::string halcon_script_filepath = "/home/i3drwl001/catkin_ws/src/i3dr_halcon_object_detect-ros/hdev/primitive_detection.hdev";

bool detect()
{
    try
    {
        HDevEngine hdev_engine;
        HDevProgram hdev_detection_program;

        try
        {
            hdev_detection_program.LoadProgram(halcon_script_filepath.c_str());
        }
        catch (HDevEngineException &hdev_exception)
        {
            std::stringstream error_ss;
            error_ss << "Problem loading halcon script" << std::endl;
            error_ss << hdev_exception.Message();
            ROS_ERROR("%s", error_ss.str().c_str());
            return false;
        }

        std::string point_cloud_filepath = "/home/i3drwl001/catkin_ws/data/point_clouds/ball001.ply";

        HTuple hVarGlobalInputPointCloudFilepath;
        hVarGlobalInputPointCloudFilepath[0] = point_cloud_filepath.c_str();

        hdev_engine.SetGlobalCtrlVarTuple("GlobalInputPointCloudFilepath", hVarGlobalInputPointCloudFilepath);

        HDevProgramCall hdev_detection_program_call = hdev_detection_program.Execute();

        HTuple hVarGlobalOutputObjectPosition;
        hVarGlobalOutputObjectPosition = hdev_engine.GetGlobalCtrlVarTuple("GlobalOutputObjectPosition");
        double x = hVarGlobalOutputObjectPosition[0].D();
        double y = hVarGlobalOutputObjectPosition[1].D();
        double z = hVarGlobalOutputObjectPosition[2].D();
        std::cout << x << ", " << y << ", " << z <<  std::endl;

        return true;
    }
    catch (HDevEngineException &hdev_exception)
    {
        std::stringstream error_ss;
        error_ss << "Problem in halcon during detection" << std::endl;
        error_ss << hdev_exception.Message();
        ROS_ERROR("%s", error_ss.str().c_str());
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    std::string ns = ros::this_node::getNamespace();

    //Check halcon script exists
    if (!boost::filesystem::exists(halcon_script_filepath))
    {
        ROS_ERROR("Failed to find halcon script for detection at: %s", halcon_script_filepath.c_str());
        return 0;
    }

    bool success = detect();
    if (!success){
        ROS_ERROR("Detection failed");
        return 0;
    }

    //ros::spin();
}