#include "safe_and_robust_lidar_map_update/update_localization.hpp" 

using namespace MapChangeDetection;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "update_localization", ros::init_options::NoRosout);

   
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");

    try {
    ROS_INFO(" Initializing node");
    UpdateLocalization UL(nh, nh_local);
    ros::spin();
    }
    catch (const char* s) {
    ROS_FATAL_STREAM(" "  << s);
    }
    catch (...) {
    ROS_FATAL_STREAM(": Unexpected error");
    }

    return 0;    
    
}
