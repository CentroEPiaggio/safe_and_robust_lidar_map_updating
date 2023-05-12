#ifndef UPDATE_LOCALIZATION_H
#define UPDATE_LOCALIZATION_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "sensor_msgs/LaserScan.h"


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>
// #include <pcl/filters/approximate_voxel_grid.h>
// #include <pcl/common/common.h>
// #include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/common/time.h>
// #include <pcl/console/print.h>
// #include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/registration/ndt.h>


#include <pcl/visualization/pcl_visualizer.h>
#include "laser_geometry/laser_geometry.h"
#include "message_filters/subscriber.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <iostream>
#include <std_srvs/Empty.h>

namespace MapChangeDetection
{

class UpdateLocalization
{
public:
  UpdateLocalization(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~UpdateLocalization();

private:
 
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    
   
    ros::Publisher update_pose_pub_;
    ros::Subscriber updated_map_sub_, laser_sub_, amcl_sub_;
    
    ros::ServiceServer serverLostLoc;
		ros::ServiceClient finish_process;

    nav_msgs::OccupancyGrid updated_map;
    sensor_msgs::LaserScan current_scan;
    geometry_msgs::PoseWithCovarianceStamped current_amcl_pose, last_loc_pose;

    int num_cell;


    //laser_geometry::LaserProjection projector_;

    
   // std::unique_ptr<tf2_ros::Buffer> tf_;
   // std::unique_ptr<tf2_ros::TransformListener> tfL_;
   // std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;

    ros::Duration transform_timeout_, tf_buffer_dur_;
    
    double tmp_val, tmp_val_1;

   // std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laser_sub_;
   // std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> laser_filters_;
    
    
    
    bool Update;
    std::string updated_map_topic_, fixed_frame_,odom_topic_,laser_topic_, pose_topic_,filter_initial_pose_topic_;
    
    void UpdatedMapCallback (const  nav_msgs::OccupancyGrid::ConstPtr& map_msg);
    void LaserCallback (const  sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void AmclCallback(const  geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    bool callbackLostLoc(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    void GetPcd(nav_msgs::OccupancyGrid map, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    bool ICP_ndt(pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud, Eigen::Matrix4d &pose, geometry_msgs::PoseWithCovarianceStamped amcl_pose);


    void BuildMap();
    ros::Time time_0,time_1;
    bool first, scan_received, map_received, enable;


};

} // end namespace

#endif //UPDATE_LOCALIZATION_H




