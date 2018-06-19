/*
 * SgbotApplication.h
 *
 *  Created on: May 9, 2018
 *      Author: cybernik
 */

#ifndef BUILD_SOURCE_LIBSGBOT_SGBOTAPPLICATION_H_
#define BUILD_SOURCE_LIBSGBOT_SGBOTAPPLICATION_H_

#include <Application/Application.h>
#include <Service/ServiceType/ServiceBase.h>
#include <Service/ServiceType/ServiceTransform.h>

#include <Service/Service.h>
#include <DataSet/DataType/OccupancyGrid.h>
#include <Service/ServiceType/ServiceMap.h>
#include <DataSet/Subscriber.h>
#include <DataSet/Publisher.h>
#include <Service/Server.h>
#include <DataSet/DataType/LaserScan.h>
#include <Parameter/Parameter.h>
#include <Service/Client.h>
#include <Service/ServiceType/ServiceOdometry.h>

#include <slam/hector/mapping.h>
#include <type/odometry.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


namespace NS_Sgbot
{
	struct _match_point{
		int count;
		int distance;
		float theta;
	};
	typedef struct _match_point MatchPoint;


	class SgbotApplication: public Application
	{
		public:
			SgbotApplication();
			virtual ~SgbotApplication();

		private:

			sgbot::slam::hector::HectorMapping *mapping;
			//NS_DataType::OccupancyGrid map;
			sgbot::Map2D map;
			sgbot::Map2D display_map;

			sgbot::Pose2D pose_;

		    NS_Service::Server< sgbot::Map2D >* map_srv;

		    NS_DataSet::Subscriber< NS_DataType::LaserScan >* laser_sub;

		    NS_Service::Server< sgbot::tf::Transform2D >* map_tf_srv;

		    NS_Service::Client< sgbot::tf::Transform2D >* odom_tf_cli;

		    NS_Service::Client<sgbot::Odometry> *odom_pose_cli;

		    NS_Service::Server<sgbot::Pose2D>* pose_srv;

		    NS_Service::Server<sgbot::Pose2D>* display_pose_srv;

		    NS_Service::Server<sgbot::Map2D>* display_map_srv;

		    NS_DataSet::Publisher<sgbot::Velocity2D>* twist_pub;

		    NS_DataSet::Publisher<int> *map_ready_pub;

		    sgbot::tf::Transform2D map_to_odom_;
		    //boost::mutex map_to_odom_lock_;
		    boost::mutex map_lock;
		    boost::thread update_map_thread;
		    boost::thread match_map_thread;

		    int update_map_level_;
		    int display_map_level_;

		    float map_update_frequency_;

			double map_resolution_;
			int map_width_;
			int map_height_;
			float map_left_offset_;
			float map_top_offset_;
			float min_update_distance_;
			float min_update_theta_;
			float update_free_factor_;
			float update_occupied_factor_;
			bool use_multi_level_maps_;
			int log_fd;
			int map_init_count;
			int map_init_step;
			int map_inited;
			NS_DataType::LaserScan laser_scan_;
			std::vector<MatchPoint> match_points_;
			MatchPoint match_point_;
			int match_point_threshold;
			float match_angular_vel;


		private:
		    void loadParameters();
		    void updateMapLoop(double frequency);
		    void mapService(sgbot::Map2D &srv_map);
		    void mapTransformService(sgbot::tf::Transform2D& transform);
		    void scanDataCallback(NS_DataType::LaserScan &scan);
		    //Eigen::Matrix3f getCovariance(sgbot::la::Matrix<float, 3, 3>& cov);
		    void poseService(sgbot::Pose2D &srv_pose);

		    void displayPoseService(sgbot::Pose2D &srv_occ_pose);

		    void displayMapService(sgbot::Map2D &srv_display_map);

		    void makeTurn(float theta);

		    void matchMap(NS_DataType::LaserScan &scan);

		    MatchPoint matchMapLaser(NS_DataType::LaserScan &scan, float theta);

		    void matchMapLoop(double frequency);


		    //void getMap(NS_DataType::OccupancyGrid& map, const sgbot::Map2D& map2d);
		public:
			virtual void
			run();
			virtual void
			quit();
	};
}


#endif /* BUILD_SOURCE_LIBSGBOT_SGBOTAPPLICATION_H_ */
