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
#include <Service/Server.h>
#include <DataSet/DataType/LaserScan.h>
#include <Parameter/Parameter.h>

#include <slam/hector/mapping.h>

namespace NS_Sgbot
{

	class SgbotApplication: public Application
	{
		public:
			SgbotApplication();
			virtual ~SgbotApplication();

		private:

			sgbot::slam::hector::HectorMapping *mapping;
			NS_DataType::OccupancyGrid map;

		    NS_Service::Server< NS_ServiceType::ServiceMap >* map_srv;

		    NS_DataSet::Subscriber< NS_DataType::LaserScan >* laser_sub;

		    NS_Service::Server< NS_ServiceType::ServiceTransform >* map_tf_srv;

		    sgbot::tf::Transform2D map_transform;

		    boost::mutex map_lock;
		    boost::thread update_map_thread;

		    int update_map_level_;

		    double map_update_frequency_;

			double map_resolution_;
			int map_width_;
			int map_height_;
			int map_left_offset_;
			int map_top_offset_;
			double min_update_distance_;
			double min_update_theta_;
			double update_free_factor_;
			double update_occupied_factor_;
			bool use_multi_level_maps_;

		private:
		    void loadParameters();
		    void updateMapLoop(double frequency);
		    void mapService(NS_ServiceType::ServiceMap &srv_map);
		    void mapTransformService(NS_ServiceType::ServiceTransform& transform);
		    void scanDataCallback(NS_DataType::LaserScan &scan);
		    //Eigen::Matrix3f getCovariance(sgbot::la::Matrix<float, 3, 3>& cov);

		    void getMap(NS_DataType::OccupancyGrid& map, const sgbot::Map2D& map2d);
		public:
			virtual void
			run();
			virtual void
			quit();
	};
}


#endif /* BUILD_SOURCE_LIBSGBOT_SGBOTAPPLICATION_H_ */
