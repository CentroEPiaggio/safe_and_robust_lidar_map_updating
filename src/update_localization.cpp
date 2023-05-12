#include "map_change_detection/update_localization.hpp" 

using namespace std;
using namespace MapChangeDetection;
typedef uint32_t index_t;
typedef int16_t coord_t;

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_initial_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_updated_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud (new pcl::PointCloud<pcl::PointXYZ>);
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
UpdateLocalization::UpdateLocalization(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
    
    serverLostLoc = nh_.advertiseService("lost_localization", &UpdateLocalization::callbackLostLoc, this); 
    
    
    nh_local_.param<string>("updated_map_topic",updated_map_topic_,"/processed_map");
    nh_local_.param<string>("global_frame",fixed_frame_,"map");
    nh_local_.param<string>("laser_topic",laser_topic_,"/scan");
    nh_local_.param<string>("global_pose_topic",pose_topic_,"/amcl_pose");
    nh_local_.param<string>("filter_initial_pose_topic",filter_initial_pose_topic_,"/initialpose");
    
    
    nh_local_.param("transform_timeout", tmp_val, 5.);
    nh_local_.param("tf_buffer_duration", tmp_val_1, 10.);

    amcl_sub_ = nh_.subscribe(pose_topic_, 100, &UpdateLocalization::AmclCallback,this);

    transform_timeout_ = ros::Duration(tmp_val);
    tf_buffer_dur_ = ros::Duration(tmp_val_1);
  
   // tf_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(tf_buffer_dur_));
    //tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
    //tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    
    //laser_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_,laser_topic_ , 10);
    //laser_filters_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(*laser_sub_, *tf_, fixed_frame_, 10, nh_);
    //laser_filters_->registerCallback(boost::bind(&UpdateLocalization::LaserCallback, this, _1));
    
    finish_process = nh_.serviceClient<std_srvs::Empty>("/publish_cmd_vel/start");
  
    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan> (laser_topic_, 1, &UpdateLocalization::LaserCallback, this);

	updated_map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid> (updated_map_topic_, 1, &UpdateLocalization::UpdatedMapCallback, this);

	update_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> (filter_initial_pose_topic_, 1);

	Update = scan_received = map_received = false;
	first = true;
	enable = false;
	time_0 = time_1 = ros::Time::now();
	BuildMap();
}



void UpdateLocalization::GetPcd(nav_msgs::OccupancyGrid map, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cloud->points.clear();
    for (coord_t x=0; x<map.info.width; x++)
    {
        for (coord_t y=0; y<map.info.height; y++)
        {
            int value = map.data[(x + y*map.info.width)];
            if ( value == 100)
            {
                pcl::PointXYZ point;
        
                point.x = (x + 0.5)*map.info.resolution + map.info.origin.position.x;
                point.y = (y + 0.5)*map.info.resolution + map.info.origin.position.y;
                point.z = 0;
				//if(sqrt((point.y - last_loc_pose.pose.pose.position.y)*(point.y - last_loc_pose.pose.pose.position.y)+(point.x - last_loc_pose.pose.pose.position.x)*(point.x - last_loc_pose.pose.pose.position.x)) < 5)
				{
	                cloud->points.push_back(point);
				}
            }
        }
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = true;
        
}

void UpdateLocalization::AmclCallback (const  geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	current_amcl_pose = *msg;
}



void UpdateLocalization::LaserCallback (const  sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    current_scan = *scan_msg;
    scan_received = true;
}

void UpdateLocalization::UpdatedMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg_map) 
{
    updated_map = *msg_map;
	map_received = true;
}

//servizio lost localization
bool UpdateLocalization::callbackLostLoc(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
    Update = true;
	last_loc_pose = current_amcl_pose;
}

bool UpdateLocalization::ICP_ndt(pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud, Eigen::Matrix4d &pose, geometry_msgs::PoseWithCovarianceStamped amcl_pose)
{
	
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_aligned_final(new pcl::PointCloud<pcl::PointNormal>);

	pcl::copyPointCloud(*target_cloud,*cloud_tgt_in);
	pcl::copyPointCloud(*lidar_cloud,*cloud_src_in);

	cloud_src_in->is_dense = false;
    cloud_tgt_in->is_dense = false;

	std::vector<int> indices1;
	pcl::removeNaNFromPointCloud(*cloud_src_in, *cloud_src_in,indices1);
	std::vector<int> indices2;
	pcl::removeNaNFromPointCloud(*cloud_tgt_in, *cloud_tgt_in,indices2);
		
		
	Eigen::Matrix4f T_tgt2src = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transformation_0 = Eigen::Matrix4f::Identity();  // source cloud centroid
	Eigen::Matrix4f transformation_4 = Eigen::Matrix4f::Identity();  // target cloud centroid
	Eigen::Vector4f c_src, c_tgt;
	Eigen::Affine3f transl_c_src = Eigen::Affine3f::Identity();
	Eigen::Affine3f transl_c_tgt = Eigen::Affine3f::Identity();
	pcl::compute3DCentroid<pcl::PointXYZ>(*cloud_src_in, c_src);
	pcl::compute3DCentroid<pcl::PointXYZ>(*cloud_tgt_in, c_tgt);
	transl_c_src.translation().matrix() = Eigen::Vector3f(c_src[0], c_src[1], c_src[2]);
	transl_c_tgt.translation().matrix() = Eigen::Vector3f(c_tgt[0], c_tgt[1], c_tgt[2]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_centr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_centr(new pcl::PointCloud<pcl::PointXYZ>);

  	pcl::transformPointCloud<pcl::PointXYZ>(*cloud_src_in, *cloud_src_centr, transl_c_src.inverse());
  	pcl::transformPointCloud<pcl::PointXYZ>(*cloud_tgt_in, *cloud_tgt_centr, transl_c_tgt.inverse());
  	transformation_0.block<3,1>(0,3) << -c_src[0], -c_src[1], -c_src[2];
  	transformation_4.block<3,1>(0,3) << c_tgt[0], c_tgt[1], c_tgt[2];

    double FitnessScore = 1.0;
	int num_iter = 1;
	int half_num_inter = ceil(num_iter / 2);
	std::vector<double> vec_scores;
	std::vector<Eigen::Matrix4d> vec_T_tgt2src;
	for(int i = 0; i < num_iter; i++) 
	{

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src_norm(new pcl::PointCloud<pcl::PointNormal>);
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_rotate_src_norm(new pcl::PointCloud<pcl::PointNormal>);
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tgt_norm(new pcl::PointCloud<pcl::PointNormal>);
		pcl::copyPointCloud(*cloud_src_centr, *cloud_src_norm);
		pcl::copyPointCloud(*cloud_tgt_centr,*cloud_tgt_norm);

		float angle;
		if(i < half_num_inter) 
			angle = (6.28/num_iter) * i; 
		else 
			angle = -(6.28/num_iter) * i;
			
		
		Eigen::Matrix4f transformation_1 = Eigen::Matrix4f::Identity();
		Eigen::Quaternionf q;
		q.w() = amcl_pose.pose.pose.orientation.w;
		Eigen::Vector3f vec(amcl_pose.pose.pose.orientation.x, amcl_pose.pose.pose.orientation.y,amcl_pose.pose.pose.orientation.z);
		q.vec() = vec;
        Eigen::Matrix3f r_tmp_t1;
		r_tmp_t1 = q.toRotationMatrix();
		transformation_1.block<3,3>(0,0) = r_tmp_t1;
		transformation_1.block<3,1>(0,3) << amcl_pose.pose.pose.position.x, amcl_pose.pose.pose.position.y,amcl_pose.pose.pose.position.z;

		pcl::transformPointCloud (*cloud_src_norm, *cloud_rotate_src_norm, transformation_1);

		// Estimate normals for source
		pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> nest_src;
		nest_src.setRadiusSearch(0.005);
		nest_src.setInputCloud(cloud_rotate_src_norm);
		nest_src.compute(*cloud_rotate_src_norm);
		
		// Estimate normals for target
		pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> nest_tgt;
		nest_tgt.setRadiusSearch(0.005);
		nest_tgt.setInputCloud(cloud_tgt_norm);
		nest_tgt.compute(*cloud_tgt_norm);

		pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgt_features(new pcl::PointCloud<pcl::FPFHSignature33>);

		// Estimate features
		pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
		fest.setRadiusSearch(0.005);
		fest.setInputCloud(cloud_rotate_src_norm);
		fest.setInputNormals(cloud_rotate_src_norm);
		fest.compute(*src_features);
		fest.setInputCloud(cloud_tgt_norm);
		fest.setInputNormals(cloud_tgt_norm);
		fest.compute(*tgt_features);

		// The Iterative Closest Point algorithm
		pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
		icp.setMaximumIterations(10000);
		icp.setEuclideanFitnessEpsilon(0.0000001);
		icp.setInputSource(cloud_rotate_src_norm);
		icp.setInputTarget(cloud_tgt_norm);


		//icp.setTransformationEpsilon (1e-9);

		cloud_aligned_final->clear();
		{
			//pcl::ScopeTime t("Alignment");
			icp.align (*cloud_aligned_final);

		}
		if (icp.hasConverged ()){
			Eigen::Matrix4f transformation_2 = icp.getFinalTransformation();
			T_tgt2src =  transformation_4 * transformation_2 * transformation_1 * transformation_0;

			FitnessScore = icp.getFitnessScore ();
			vec_scores.push_back(FitnessScore);
			vec_T_tgt2src.push_back(T_tgt2src.cast <double> ());
		}
		else
		{
			FitnessScore = 100;
			pcl::console::print_error ("Alignment failed!\n");
		}
	}

	int minValueIndex = std::min_element(vec_scores.begin(),vec_scores.end()) - vec_scores.begin();
	double minValue = *std::min_element(vec_scores.begin(), vec_scores.end());

	Eigen::Matrix4d finale_transformation = vec_T_tgt2src[minValueIndex];


		pcl::console::print_info("    | %6.3f %6.3f %6.3f %6.3f | \n", finale_transformation(0,0), finale_transformation(0,1), finale_transformation(0,2), finale_transformation(0,3));
		pcl::console::print_info("T = | %6.3f %6.3f %6.3f %6.3f | \n", finale_transformation(1,0), finale_transformation(1,1), finale_transformation(1,2), finale_transformation(1,3));
		pcl::console::print_info("    | %6.3f %6.3f %6.3f %6.3f | \n", finale_transformation(2,0), finale_transformation(2,1), finale_transformation(2,2), finale_transformation(2,3));
		pcl::console::print_info("    | %6.3f %6.3f %6.3f %6.3f | \n", finale_transformation(3,0), finale_transformation(3,1), finale_transformation(3,2), finale_transformation(3,3));
		pcl::console::print_info("\n"); 
		std::cout << "FitnessScore: " << minValue << std::endl;
		pose = vec_T_tgt2src[minValueIndex];

		return true;
	
}
void UpdateLocalization::BuildMap()
{
	ros::Rate rate(20.0);
	while (ros::ok())
    {
		if (Update)
         {

			for(int i = 0; i <  current_scan.ranges.size(); i++)
			{
				if(current_scan.ranges[i] >= 15)current_scan.ranges[i]= NAN;
			}

            //from laser data to pcl
            scan_cloud->width = current_scan.ranges.size();
            scan_cloud->height = 1;
            scan_cloud->points.resize(scan_cloud->width * scan_cloud->height);

            for(int i = 0; i < scan_cloud->width; i ++)
            {
                double angle = current_scan.angle_min + (i * current_scan.angle_increment);
                scan_cloud->points[i].x = current_scan.ranges[i] * cos(angle);
                scan_cloud->points[i].y = current_scan.ranges[i] * sin(angle);
                scan_cloud->points[i].z = 0.0;
            }
            scan_cloud->is_dense = false;
			std::vector<int> ind;
			pcl::removeNaNFromPointCloud(*scan_cloud, *scan_cloud, ind);

			
            //Updated_map_cloud
            GetPcd(updated_map,pcl_updated_cloud);
      	
            Eigen::Matrix4d pose(Eigen::Matrix4d::Identity());
            std::cout<<"ICP_start"<<std::endl;
            ICP_ndt(pcl_updated_cloud,scan_cloud, pose,last_loc_pose);
            std::cout<<"ICP_end"<<std::endl;


            geometry_msgs::PoseWithCovarianceStamped update_filter_pose;
            Eigen::Quaterniond quat(pose.block<3,3>(0,0));
            
            update_filter_pose.pose.pose.orientation.w = quat.w();
            update_filter_pose.pose.pose.orientation.x = quat.x();
            update_filter_pose.pose.pose.orientation.y = quat.y();
            update_filter_pose.pose.pose.orientation.z = quat.z();
            update_filter_pose.pose.pose.position.x = pose(0,3);
            update_filter_pose.pose.pose.position.y = pose(1,3);
            update_filter_pose.pose.pose.position.z = pose(2,3);

            update_filter_pose.header.frame_id =  fixed_frame_;
		    update_filter_pose.header.stamp = ros::Time::now();
		    update_pose_pub_.publish(update_filter_pose);
            Update = false;
			std_srvs::Empty req_srv;
			finish_process.call(req_srv);
			
			
			}
		ros::spinOnce();
      	rate.sleep();
	}
		
}	

UpdateLocalization::~UpdateLocalization() {

}


