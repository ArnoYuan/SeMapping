/*
 * SgbotApplication.cpp
 *
 *  Created on: May 9, 2018
 *      Author: cybernik
 */
#include "SgbotApplication.h"

#define USE_DBG
#ifdef USE_DBG
#include <stdio.h>
#define DBG_PRINTF	printf
#else
#define DBG_PRINTF
#endif
#include <stdio.h>
#include <Time/Time.h>


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
	    odom_tf_cli = new NS_Service::Client< NS_ServiceType::ServiceTransform >(
	        "BASE_ODOM_TF");

	    odom_pose_cli = new NS_Service::Client< NS_ServiceType::ServiceOdometry>(
	    		"BASE_ODOM");
	}

	SgbotApplication::~SgbotApplication()
	{
		delete mapping;
		delete map_srv;
		delete laser_sub;
		delete map_tf_srv;
		delete odom_tf_cli;
		close(log_fd);
		running = false;
		update_map_thread.join();
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
		update_free_factor_ = parameter.getParameter("update_free_factor", 0.4f);
		update_occupied_factor_ = parameter.getParameter("update_occupied_factor", 0.9f);
		min_update_theta_ = parameter.getParameter("min_udpate_theta", 0.9f);
		min_update_distance_ = parameter.getParameter("min_udpate_distance", 0.4f);
/*
		DBG_PRINTF("-----------------------\n");
		DBG_PRINTF("update_map_level:%d\n", update_map_level_);
		DBG_PRINTF("map_update_frequency:%f\n", map_update_frequency_);
		DBG_PRINTF("map_resolution:%f\n", map_resolution_);
		DBG_PRINTF("map_height:%d\n", map_height_);
		DBG_PRINTF("map_width:%d\n", map_width_);
		DBG_PRINTF("left_offset:%f\n", map_left_offset_);
		DBG_PRINTF("top_offset:%f\n", map_top_offset_);
		DBG_PRINTF("use_multi_level_maps:%d\n",use_multi_level_maps_);
		DBG_PRINTF("update_free_factor:%f\n", update_free_factor_);
		DBG_PRINTF("update_occupied_factor:%f\n", update_occupied_factor_);
		DBG_PRINTF("min_update_theta:%f\n", min_update_theta_);
		DBG_PRINTF("min_update_distance:%f\n", min_update_distance_);
		DBG_PRINTF("-----------------------\n");
		*/
	}

	static inline void transformTFToMsg(const sgbot::tf::Transform2D& bt,
	                                      NS_DataType::Transform& msg)
	{
		float x=0;
		float y = 0;
		float theta = 0;
		float scalar = 0;
		bt.getValue(x, y, theta, scalar);

		msg.translation.x = x;
		msg.translation.y = y;
		msg.translation.z = 0;
		msg.rotation.x = 0;
		msg.rotation.y = 0;
		msg.rotation.z = sin(theta/2);
		msg.rotation.w = cos(theta/2);

	}

	static inline void transformMsgToTF(const NS_DataType::Transform& msg, sgbot::tf::Transform2D& bt)
	{
		float x = 0;
		float y = 0;
		float theta = 0;
		float scalar = 1;
		x = msg.translation.x;
		y = msg.translation.y;
		theta = asin(msg.rotation.z)*2;
		bt.setValue(x, y, theta, scalar);
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
		boost::mutex::scoped_lock map_tf_mutex(map_to_odom_lock_);
		transformTFToMsg(map_to_odom_, transform.transform);
	}

	void SgbotApplication::scanDataCallback(NS_DataType::LaserScan &scan)
	{
		sgbot::sensor::Lidar2D laser;

		if(!running)
			return;
		NS_NaviCommon::Time timestamp =  NS_NaviCommon::Time::now();
		laser.clear();
		sgbot::Point2D origin;
		origin.x() = 0.0f;
		origin.y() = 0.0f;

		laser.setOrigin(origin);
		float angle = scan.angle_min;
/*
		DBG_PRINTF("------------------laser--------------\n");
		DBG_PRINTF("angle_min:%f\n", scan.angle_min);
		DBG_PRINTF("angle_increment:%f\n", scan.angle_increment);
		DBG_PRINTF("ranges:%d\n", scan.ranges.size());
*/
		for(int i=0;i<scan.ranges.size();++i)
		{
			float dist = scan.ranges[i];
			//dist = 1;
			if((dist > scan.range_min) && (dist<(scan.range_max-0.1f)))
			{
				laser.addBeam(angle, dist);
				//DBG_PRINTF("(%f,%f),", angle, dist);
			}
			angle += scan.angle_increment;
		}
		mapping->updateByScan(laser);
		DBG_PRINTF("[scanDataCallback]%f\n", (NS_NaviCommon::Time::now()-timestamp).toSec());
		//DBG_PRINTF("[scanDataCallback]%f\n", timestamp.toSec());
		/*
		DBG_PRINTF("\n");
		DBG_PRINTF("-------------------------------------\n");
		*/
		//std::cout<<laser;

		/*
		boost::mutex::scoped_lock map_tf_mutex(map_transform_lock);
		sgbot::Pose2D pose = mapping->getPose();
		sgbot::la::Matrix<float, 3, 3> pose_cov = mapping->getPoseCovariance();

		map_transform_.setValue(pose.x(), pose.y(), pose.theta(), 1);
	*/
	    /*
	     * process map->odom transform
	     */
		sgbot::tf::Transform2D odom_to_base;

	    NS_ServiceType::ServiceTransform base_odom_tf;

	    if(odom_tf_cli->call(base_odom_tf))
	    {
	      boost::mutex::scoped_lock map_mutex(map_to_odom_lock_);

	      transformMsgToTF(base_odom_tf.transform, odom_to_base);
	      sgbot::Pose2D pose = mapping->getPose();
	      sgbot::tf::Transform2D map_to_base = sgbot::tf::Transform2D(pose.x(), pose.y(), pose.theta(), 1);

	      map_to_odom_ = sgbot::tf::Transform2D(map_to_base * odom_to_base.inverse());
	    }
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
					DBG_PRINTF("update map\n");

					/*
					 * log
					 */
					sgbot::Pose2D pose = mapping->getPose();
					sgbot::la::Matrix<float , 3, 3> cov = mapping->getPoseCovariance();
					NS_ServiceType::ServiceOdometry odom_pose;
					odom_pose_cli->call(odom_pose);
					int ret = 0;
					char buf[256];
					ret += sprintf(buf+ret, "[%f,%f,%f]", pose.x(), pose.y(), pose.theta());
					ret += sprintf(buf+ret, "[%f,%f,%f]",
								odom_pose.odom.pose.position.x,
								odom_pose.odom.pose.position.y,
								2*sgbot::math::asin(odom_pose.odom.pose.orientation.z));
					ret += sprintf(buf+ret, "[%f,%f]\n", odom_pose.odom.twist.linear.x, odom_pose.odom.twist.angular.z);
					write(log_fd, buf, ret);

				}
			}
			r.sleep();
		}
	}

	void SgbotApplication::run()
	{
		loadParameters();

		log_fd = open("/tmp/sgbot_log.txt", O_RDWR|O_CREAT);
		if(log_fd<0)
		{
			printf("sgbot_log file open failed.\n");
		}

		sgbot::slam::hector::HectorMappingConfig config;

		config.map_properties.resolution = map_resolution_;
		config.map_properties.height = map_height_;
		config.map_properties.width = map_width_;
		config.map_properties.left_offset = map_left_offset_;
		config.map_properties.top_offset = map_top_offset_;
		config.use_multi_level_maps = use_multi_level_maps_;
		config.update_free_factor = update_free_factor_;
		config.update_occupied_factor = update_occupied_factor_;
		config.min_update_theta = min_update_theta_;
		config.min_update_distance = min_update_distance_;
		DBG_PRINTF("map_resolution:%f", map_resolution_);
		DBG_PRINTF("map_height:%d\n", map_height_);
		DBG_PRINTF("map_width:%d\n", map_width_);
		DBG_PRINTF("left_offset:%f\n", map_left_offset_);
		DBG_PRINTF("top_offset:%f\n", map_top_offset_);
		DBG_PRINTF("use_multi_level_maps:%d\n",use_multi_level_maps_);
		DBG_PRINTF("update_free_factor:%f\n", update_free_factor_);
		DBG_PRINTF("update_occupied_factor:%f\n", update_occupied_factor_);
		DBG_PRINTF("min_update_theta:%f\n", min_update_theta_);
		DBG_PRINTF("min_update_distance:%f\n", min_update_distance_);

		mapping = new sgbot::slam::hector::HectorMapping(config);
		//mapping = new sgbot::slam::hector::HectorMapping();
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



