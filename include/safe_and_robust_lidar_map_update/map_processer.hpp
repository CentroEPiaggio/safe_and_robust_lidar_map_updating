#include "grid.hpp"

#include <safe_and_robust_lidar_map_update/ChangedCells.h>

class MapProcesser
{
	public:
		MapProcesser(ros::NodeHandle* nodehandle);

	private:

		ros::NodeHandle nh_;
		ros::Subscriber	sub;
		ros::Publisher	pub;

		nav_msgs::OccupancyGrid initial_map;

		void subscriberCallback(const safe_and_robust_lidar_map_update::ChangedCells& msg);
};