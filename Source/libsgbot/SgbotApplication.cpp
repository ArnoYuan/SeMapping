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

#define DEG2RAD(x) ((x)*M_PI/180.)
#define RAD2DEG(x) ((x)*180./M_PI)

#define USE_POSE_LOG

namespace NS_Sgbot
{
	SgbotApplication::SgbotApplication()
	{


		laser_sub = new NS_DataSet::Subscriber<NS_DataType::LaserScan>(
				"LASER_SCAN",
				boost::bind(&SgbotApplication::scanDataCallback, this, _1));
	    map_tf_srv = new NS_Service::Server< sgbot::tf::Transform2D >(
	        "ODOM_MAP_TF",
	        boost::bind(&SgbotApplication::mapTransformService, this, _1));
	    odom_tf_cli = new NS_Service::Client< sgbot::tf::Transform2D >(
	        "BASE_ODOM_TF");

	    odom_pose_cli = new NS_Service::Client< sgbot::Odometry >(
	    		"BASE_ODOM");

		map_srv = new NS_Service::Server<sgbot::Map2D>(
				"MAP", boost::bind(&SgbotApplication::mapService, this, _1));

		pose_srv = new NS_Service::Server<sgbot::Pose2D>(
				"POSE", boost::bind(&SgbotApplication::poseService, this, _1));

		display_pose_srv = new NS_Service::Server<sgbot::Pose2D>(
				"DISPALY_POSE", boost::bind(&SgbotApplication::displayPoseService, this, _1));

		display_map_srv = new NS_Service::Server<sgbot::Map2D>(
				"DISPLAY_MAP", boost::bind(&SgbotApplication::displayMapService, this, _1));

		twist_pub = new NS_DataSet::Publisher<sgbot::Velocity2D>(
				"TWIST");

		map = sgbot::Map2D();

	}

	SgbotApplication::~SgbotApplication()
	{
		delete mapping;
		delete map_srv;
		delete laser_sub;
		delete map_tf_srv;
		delete odom_tf_cli;
		delete pose_srv;
#ifdef USE_POSE_LOG
		close(log_fd);
#endif
		running = false;
		update_map_thread.join();
	}

	void SgbotApplication::loadParameters()
	{
		NS_NaviCommon::Parameter parameter;

		parameter.loadConfigurationFile("sgbot_mapping.xml");

		update_map_level_ = parameter.getParameter("update_map_level", 1);
		display_map_level_ = parameter.getParameter("display_map_level", 0);
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



	void SgbotApplication::mapService(sgbot::Map2D& srv_map)
	{
		if(!running)
			return;
		boost::mutex::scoped_lock map_mutex(map_lock);
		printf("map service...\n");
		srv_map = map;
		/*
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
		*/
	}

	void SgbotApplication::displayMapService(sgbot::Map2D& srv_display_map)
	{
		boost::mutex::scoped_lock map_mutex(map_lock);
		srv_display_map = display_map;
	}

	void SgbotApplication::mapTransformService(sgbot::tf::Transform2D& transform)
	{
		//boost::mutex::scoped_lock map_tf_mutex(map_to_odom_lock_);

		sgbot::tf::Transform2D odom_to_base;

	    //NS_ServiceType::ServiceTransform base_odom_tf;
		sgbot::tf::Transform2D base_odom_tf;

	    if(odom_tf_cli->call(base_odom_tf))
	    {
	      boost::mutex::scoped_lock map_mutex(map_lock);

	     // transformMsgToTF(base_odom_tf.transform, odom_to_base);
	      sgbot::Pose2D pose = mapping->getPose();
	      sgbot::Point2D origin = map.getOrigin();
	      pose_ = pose;
	      sgbot::tf::Transform2D map_to_base = sgbot::tf::Transform2D(pose.x()-origin.x(), pose.y()-origin.y(), pose.theta(), 1);

	      map_to_odom_ = sgbot::tf::Transform2D(map_to_base * odom_to_base.inverse());

	    }
	    transform = map_to_odom_;
	}

	void SgbotApplication::scanDataCallback(NS_DataType::LaserScan &scan)
	{
		sgbot::sensor::Lidar2D laser;

		if(!running)
			return;

		if(map_inited)
		{
			NS_NaviCommon::Time timestamp =  NS_NaviCommon::Time::now();
			laser.clear();
			sgbot::Point2D origin;
			origin.x() = 0.0f;
			origin.y() = 0.0f;

			laser.setOrigin(origin);
			float angle = scan.angle_min;

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
		}
		else
		{
			boost::mutex::scoped_lock map_mutex(map_lock);
			if(map_init_count==19)
				laser_scan_ = scan;
			map_init_count++;
			DBG_PRINTF("laser init scan :%d\n", map_init_count);

		}
	    /*
	     * process map->odom transform
	     */
#if 0
		sgbot::tf::Transform2D odom_to_base;

	    //NS_ServiceType::ServiceTransform base_odom_tf;
		sgbot::tf::Transform2D base_odom_tf;

	    if(odom_tf_cli->call(base_odom_tf))
	    {
	      boost::mutex::scoped_lock map_mutex(map_to_odom_lock_);

	     // transformMsgToTF(base_odom_tf.transform, odom_to_base);
	      sgbot::Pose2D pose = mapping->getPose();
	      pose_ = pose;
	      sgbot::tf::Transform2D map_to_base = sgbot::tf::Transform2D(pose.x(), pose.y(), pose.theta(), 1);

	      map_to_odom_ = sgbot::tf::Transform2D(map_to_base * odom_to_base.inverse());
	    }
#endif
	}
	void SgbotApplication::poseService(sgbot::Pose2D &srv_pose)
	{
		if(!running)
			return;
		//boost::mutex::scoped_lock map_mutex(map_lock);
		srv_pose = mapping->getPose();
		DBG_PRINTF("[poseService][%f,%f,%f]\n", srv_pose.x(), srv_pose.y(), srv_pose.theta());
		//srv_pose = pose_;
	}

	void SgbotApplication::displayPoseService(sgbot::Pose2D &srv_display_pose)
	{
		if(!running)
			return;
		boost::mutex::scoped_lock map_mutex(map_lock);
		sgbot::Pose2D pose = mapping->getPose();

		sgbot::Point2D origin = display_map.getOrigin();
		pose.x() = (pose.x()-origin.x())/display_map.resolution_;
		pose.y() = (pose.y()-origin.y())/display_map.resolution_;
		srv_display_pose = pose;
	}

#if 0
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
#endif

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
					map = map2d;
					//getMap(map, map2d);
					DBG_PRINTF("update map\n");
#ifdef USE_POSE_LOG
					/*
					 * log
					 */
					sgbot::Pose2D pose = mapping->getPose();
					sgbot::la::Matrix<float , 3, 3> cov = mapping->getPoseCovariance();
					sgbot::Odometry odom_pose;
					if(odom_pose_cli->call(odom_pose))
					{
						int ret = 0;
						char buf[256];
						ret += sprintf(buf+ret, "[%f,%f,%f]", pose.x(), pose.y(), pose.theta());
						ret += sprintf(buf+ret, "[%f,%f,%f]",
									odom_pose.pose2d.x(), odom_pose.pose2d.y(), odom_pose.pose2d.theta());
						ret += sprintf(buf+ret, "[%f,%f]\n", odom_pose.velocity2d.linear, odom_pose.velocity2d.angular);
						write(log_fd, buf, ret);
					}
#endif
				}
				if(mapping->hasUpdatedMap(display_map_level_))
				{
					display_map = mapping->getMap(display_map_level_);
				}
			}
			r.sleep();
		}
	}

	void SgbotApplication::matchMapLoop(double frequency)
	{
		NS_NaviCommon::Rate r(frequency);
		while(running)
		{
			if(map_inited==0)
			{
				boost::mutex::scoped_lock map_mutex(map_lock);
				DBG_PRINTF("map_init_count=%d", map_init_count);
				if(map_init_count>=20)
				{
					DBG_PRINTF("matchMap ...\n");
					matchMap(laser_scan_);
					map_inited = 1;
					return;
				}
			}
		}
	}

	MatchPoint SgbotApplication::matchMapLaser(NS_DataType::LaserScan &scan, float theta)
	{

		std::vector<int> x_values;
		int i = 0;
		x_values.resize(map_width_);
		for(i=0;i<x_values.size();i++)
		{
			x_values[i]=0;
		}
		float angle = scan.angle_min+theta;

		for(i=0;i<scan.ranges.size();i++)
		{
			float distance = scan.ranges[i];
			if(distance>scan.range_min&&distance<scan.range_max)
			{
				int x = distance*sgbot::math::cos(angle)/map_resolution_+map_width_/2;

				x_values[x]++;
			}
			angle += scan.angle_increment;
			if(angle<-M_PI)
				angle+=2*M_PI;

			if(angle>M_PI)
				angle-=2*M_PI;
		}
		int index = 0;
		int max_value = 0;
		max_value = x_values[0];
		for(i=0;i<x_values.size();i++)
		{
			if(max_value<x_values[i])
			{
				max_value = x_values[i];
				index = i;
			}
		}
		MatchPoint match_point;
		match_point.count = max_value;
		match_point.theta = theta;
		match_point.distance = index-map_width_/2;
		return match_point;
	}

	void SgbotApplication::makeTurn(float theta)
	{
		sgbot::Odometry origin_odom;
		sgbot::Velocity2D velocity2d;
		odom_pose_cli->call(origin_odom);
		float end_theta = origin_odom.pose2d.theta()+theta;
		DBG_PRINTF("[origin:%f][end_theta=%f]\n", RAD2DEG(origin_odom.pose2d.theta()), RAD2DEG(end_theta));
		for(;;)
		{
			sgbot::Odometry odom;
			DBG_PRINTF("make turn...\n");
			if(odom_pose_cli->call(odom))
			{
				DBG_PRINTF("odom pose cli\n");
				if(theta>0)
				{
					velocity2d.angular = 0.4;
					velocity2d.linear = 0;
					twist_pub->publish(velocity2d);
					if(origin_odom.pose2d.theta()>0)
					{
						if(odom.pose2d.theta()<0)
							odom.pose2d.theta()+=2*M_PI;
					}

					if(odom.pose2d.theta()>=end_theta)
					{
						DBG_PRINTF("[cur theta=%f]\n", RAD2DEG(odom.pose2d.theta()));
						for(;;)
						{
							if(odom_pose_cli->call(odom))
							{
								if(odom.velocity2d.angular==0&&odom.velocity2d.linear==0)
								{

									return;
								}
							}

							velocity2d.angular = 0;
							velocity2d.linear = 0;
							twist_pub->publish(velocity2d);
							NS_NaviCommon::delay(10);
						}
					}
				}
				else
				{
					velocity2d.angular = -0.4;
					velocity2d.linear = 0;
					twist_pub->publish(velocity2d);
					if(origin_odom.pose2d.theta()<0)
					{
						if(odom.pose2d.theta()>0)
							odom.pose2d.theta()-=2*M_PI;
					}
					if(odom.pose2d.theta()<=end_theta)
					{
						DBG_PRINTF("[cur theta=%f]\n", RAD2DEG(odom.pose2d.theta()));
						for(;;)
						{
							if(odom_pose_cli->call(odom))
							{
								if(odom.velocity2d.angular==0&&odom.velocity2d.linear==0)
								{
									return;
								}
							}

							velocity2d.angular = 0;
							velocity2d.linear = 0;
							twist_pub->publish(velocity2d);
							NS_NaviCommon::delay(10);
						}
					}
				}
			}
			NS_NaviCommon::delay(10);
		}
	}

	void SgbotApplication::matchMap(NS_DataType::LaserScan &scan)
	{
		std::vector<MatchPoint> points;
		points.resize(0);
		float end_theta=DEG2RAD(180);
		float delta_theta = DEG2RAD(1);
		float theta=0;

		for(theta=0;theta<end_theta;theta+=delta_theta)
		{
			points.push_back(matchMapLaser(scan, theta));
		}
		MatchPoint match_point=points[0];

		for(int i=0;i<points.size();i++)
		{
			if(match_point.count<points[i].count)
			{
				match_point = points[i];
			}
		}
		DBG_PRINTF("[count=%d][theta=%f][distance=%d]\n", match_point.count, RAD2DEG(match_point.theta), match_point.distance);
		float turn_theta = sgbot::math::fabs(match_point.theta);
		if(theta>0)
		{
			if(match_point.distance<0)
				turn_theta = M_PI-turn_theta;
			else
				turn_theta = -turn_theta;
		}
		else
		{
			if(match_point.distance<0)
				turn_theta = turn_theta-M_PI;
		}
		DBG_PRINTF("turn theta =%f\n", RAD2DEG(turn_theta));
		makeTurn(turn_theta);
		DBG_PRINTF("search wall ok!");
	}

	void SgbotApplication::run()
	{
		loadParameters();
#ifdef USE_POSE_LOG
		log_fd = open("/tmp/sgbot_log.txt", O_RDWR|O_CREAT);
		if(log_fd<0)
		{
			printf("sgbot_log file open failed.\n");
		}
#endif
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
		map_inited = 0;
		map_init_count = 0;
		update_map_thread = boost::thread(
				boost::bind(&SgbotApplication::updateMapLoop, this, map_update_frequency_));
		match_map_thread = boost::thread(boost::bind(&SgbotApplication::matchMapLoop, this, 100));
	}

	void SgbotApplication::quit()
	{
		console.message("sgbot is quitting!");
		running = false;

		update_map_thread.join();
	}
}



