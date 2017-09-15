/*
 * HectorMappingApplication.h
 *
 *  Created on: 2016年10月11日
 *      Author: seeing
 */

#ifndef _HECTORMAPPINGAPPLICATION_H_
#define _HECTORMAPPINGAPPLICATION_H_

#include <Application/Application.h>
#include <DataSet/DataType/DataBase.h>
#include <Transform/LinearMath/Transform.h>
#include <DataSet/DataType/LaserScan.h>
#include <DataSet/DataType/OccupancyGrid.h>
#include <Time/Time.h>
#include <Time/Duration.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/thread/mutex.hpp>

#include "Scan/DataPointContainer.h"
#include "Utils/HectorDebugInfoInterface.h"
#include "Utils/HectorDrawings.h"
#include "Slam/HectorSlamProcessor.h"
#include "Utils/PoseInfoContainer.h"
#include "Utils/HectorMapMutex.h"

#include <DataSet/DataType/Point32.h>
#include <Service/ServiceType/ServiceBase.h>

#include <Service/Service.h>
#include <Service/ServiceType/ServiceMap.h>
#include <Service/ServiceType/ServiceTransform.h>
#include <DataSet/DataType/Odometry.h>
#include <Service/Server.h>
#include <DataSet/Subscriber.h>
#include <Service/Client.h>

namespace NS_HectorMapping
{
  
  class HectorMappingApplication: public Application
  {
  public:
    HectorMappingApplication ();
    virtual
    ~HectorMappingApplication ();
  private:
    HectorDebugInfoInterface* debugInfoProvider;

    HectorDrawings* hectorDrawings;

    int lastGetMapUpdateIndex;

    PoseInfoContainer poseInfoContainer_;

    HectorSlamProcessor* slamProcessor;
    DataContainer laserScanContainer;

    Eigen::Vector3f lastSlamPose;

    void
    getMapInfo (NS_DataType::OccupancyGrid& map,
                const NS_HectorMapping::GridMap& gridMap);

    bool
    pointsToDataContainer (const std::vector<NS_DataType::Point32>& points,
                           const NS_Transform::StampedTransform& laserTransform,
                           DataContainer& dataContainer, float scaleToMap);

    void
    projectLaser (const NS_DataType::LaserScan& scan_in,
                  std::vector<NS_DataType::Point32>& points,
                  double range_cutoff, bool preservative);

    const boost::numeric::ublas::matrix<double>&
    getUnitVectors (double angle_min, double angle_max, double angle_increment,
                    unsigned int length);

    bool
    laserScanToDataContainer (const NS_DataType::LaserScan& scan,
                              DataContainer& dataContainer, float scaleToMap);

    void
    updateMap (NS_DataType::OccupancyGrid& map,
               const NS_HectorMapping::GridMap& gridMap,
               MapLockerInterface* mapMutex);

  private:
    //Parameters related to publishing the scanmatcher pose directly via tf
    
    double update_factor_free_;
    double update_factor_occupied_;
    double map_update_distance_threshold_;
    double map_update_angle_threshold_;

    double map_resolution_;
    int map_size_;
    double map_start_x_;
    double map_start_y_;
    int map_multi_res_levels_;

    double map_pub_period_;

    bool use_tf_scan_transformation_;
    bool use_tf_pose_start_estimate_;
    bool map_with_known_poses_;
    bool timing_output_;

    float sqr_laser_min_dist_;
    float sqr_laser_max_dist_;
    float laser_z_min_value_;
    float laser_z_max_value_;

    double map_update_frequency_;

  private:
    std::map<std::string, boost::numeric::ublas::matrix<double>*> unit_vector_map;
    boost::mutex guv_mutex;
  private:
    void
    loadParameters ();

    void
    laserDataCallback (NS_DataType::LaserScan& laser);

    void
    mapService (NS_ServiceType::ServiceMap& srv_map);

    void
    mapTransformService (NS_ServiceType::ServiceTransform& transform);

    void
    updateMapLoop (double frequency);

  private:
    NS_Transform::Transform map_to_odom;

    NS_Transform::Transform centered_laser_pose;

    int laser_count;

    NS_DataType::OccupancyGrid map;

    boost::mutex map_to_odom_lock;
    boost::mutex map_lock;

    boost::thread update_map_thread;

    NS_Service::Server<NS_ServiceType::ServiceTransform>* map_tf_srv;

	NS_Service::Server<NS_ServiceType::ServiceMap>* map_srv;

	NS_DataSet::Subscriber<NS_DataType::LaserScan>* laser_sub;

	NS_Service::Client<NS_ServiceType::ServiceTransform>* odom_tf_cli;

  public:
    virtual void
    run ();
    virtual void
    quit ();
  };

}

#endif
