/*
 * SgbotApplication.cpp
 *
 *  Created on: May 9, 2018
 *      Author: cybernik
 */
#include "SgbotApplication.h"


namespace NS_Sgbot
{
	SgbotApplication::SgbotApplication()
	{
		map_srv = new NS_Service::Server<NS_ServiceType::ServiceMap>(
				"MAP", boost::bind(&SgbotApplication::mapService, this, _1));
		laser_sub = new NS_DataSet::Subscriber<NS_DataType::LaserScan>(
				"LASER_SCAN",
				boost::bind(&SgbotApplication::scanDataCallback, this, _1));
	    map_tf_srv = new NS_Service::Server< NS_ServiceType::ServiceTransform >(
	        "ODOM_MAP_TF",
	        boost::bind(&SgbotApplication::mapTransformService, this, _1));

	}

	SgbotApplication::~SgbotApplication()
	{
		delete mapping;
		delete map_srv;
		delete laser_sub;
	}

	void SgbotApplication::loadParameters()
	{
		NS_NaviCommon::Parameter parameter;

		parameter.loadConfigurationFile("sgbot_mapping.xml");

		update_map_level_ = parameter.getParameter("update_map_level", 1);
		map_update_frequency_ = parameter.getParameter("map_update_frequency", 2.0f);
		map_resolution_ = parameter.getParameter("map_resolution", 0.025f);
		map_width_ = parameter.getParameter("map_width", 1024);
		map_height_ = parameter.getParameter("map_height", 1024);
		map_left_offset_ = parameter.getParameter("map_left_offset", 0.5f);
		map_top_offset_ = parameter.getParameter("map_top_offset", 0.5f);
		use_multi_level_maps_ = parameter.getParameter("use_mutli_level_maps", 1);
		update_free_factor_ = parameter.getParameter("update_free_factor", 0.9f);
		update_occupied_factor_ = parameter.getParameter("update_occupied_factor", 0.9f);
		min_update_theta_ = parameter.getParameter("min_udpate_theta", 0.9f);
		min_update_distance_ = parameter.getParameter("min_udpate_distance", 0.4f);
	}

	void SgbotApplication::mapService(NS_ServiceType::ServiceMap &srv_map)
	{
		boost::mutex::scoped_lock map_mutex(map_lock);
		if(map.info.width&&map.info.height)
		{
			srv_map.map = map;
			srv_map.result = true;
		}
		else
		{
			console.warning("Get map failure!");
			srv_map.result = false;
		}
	}

	void SgbotApplication::mapTransformService(NS_ServiceType::ServiceTransform& transform)
	{


	}

	void SgbotApplication::scanDataCallback(NS_DataType::LaserScan &scan)
	{
		sgbot::sensor::Lidar2D laser;

		laser.clear();
		sgbot::Point2D origin;
		origin.x() = 0.0f;
		origin.y() = 0.0f;

		laser.setOrigin(origin);
		float angle = scan.angle_min;

		for(int i=0;i<scan.ranges.size();++i)
		{
			float dist = scan.ranges[i];
			if((dist > scan.range_min) && (dist<(scan.range_max-0.1f)))
			{
				laser.addBeam(angle, dist);
			}
			angle += scan.angle_increment;
		}
		mapping->updateByScan(laser);


		sgbot::Pose2D pose = mapping->getPose();

	}

	void SgbotApplication::getMap(NS_DataType::OccupancyGrid& map, const sgbot::Map2D& map2d)
	{
		int sizeX = map2d.getWidth();
		int sizeY = map2d.getHeight();
		int size = sizeX*sizeY;
		sgbot::Point2D origin = map2d.getOrigin();
		map.info.origin.position.x = origin.x();
		map.info.origin.position.y = origin.y();
		map.info.origin.orientation.w = 1.0f;
		map.info.resolution = map2d.getResolution();

		map.info.width = sizeX;
		map.info.height = sizeY;
		map.header.frame_id = "map";

		map.data.clear();
		map.data.resize(size);
		memset(&map.data[0], -1, sizeof(int8_t)*size);

		for(int y=0;y<map.info.height;++y)
		{
			int rows = y*map.info.width;
			for(int x=0;x<map.info.width;++x)
			{

				if(map2d.isKnown(x,y))
				{
					map.data[rows+x]=0;
				}
				else if(map2d.isEdge(x,y))
				{
					map.data[rows+x]=100;
				}
			}
		}
	}

	void SgbotApplication::updateMapLoop(double frequency)
	{
		NS_NaviCommon::Rate r(frequency);
		while(running)
		{
			{
				boost::mutex::scoped_lock map_mutex(map_lock);
				if(mapping->hasUpdatedMap(update_map_level_))
				{
					sgbot::Map2D map2d = mapping->getMap(update_map_level_);
					getMap(map, map2d);
				}
			}
			r.sleep();
		}
	}

	void SgbotApplication::run()
	{
		loadParameters();

		sgbot::slam::hector::HectorMappingConfig config;
		config.map_properties.resolution = map_resolution_;
		config.map_properties.height = map_height_;
		config.map_properties.height = map_height_;
		config.map_properties.left_offset = map_left_offset_;
		config.map_properties.top_offset = map_top_offset_;
		config.min_update_distance = min_update_distance_;
		config.min_update_theta = min_update_theta_;
		config.update_free_factor = update_free_factor_;
		config.update_occupied_factor = update_occupied_factor_;
		config.use_multi_level_maps = use_multi_level_maps_;

		mapping = new sgbot::slam::hector::HectorMapping(config);

		running = true;
		update_map_thread = boost::thread(
				boost::bind(&SgbotApplication::updateMapLoop, this, map_update_frequency_));
	}

	void SgbotApplication::quit()
	{
		console.message("sgbot is quitting!");
		running = false;
		update_map_thread.join();
	}
}



