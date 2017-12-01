#include "robot_center_mapping.h"
#include <opencv2/highgui/highgui.hpp>

#define TIGEMA 0.3
using namespace  robot_center_mapping;

Center_map::Center_map(ros::NodeHandle nodehandle)
    : nodehandle_ (nodehandle),
      rawMap_  ({"elevation", "road", "variance", "front_object", "back_object"}),
      fusedMap_({"elevation", "road", "variance", "front_object", "back_object"})
{
    readParameters();

    rawMap_.setFrameId("vehicle");
    rawMap_.setGeometry(Length(mapLength_, mapLength_), mapResolution_);


    MapRawPublisher_        = nodehandle_.advertise<grid_map_msgs::GridMap>     (pubRawMap_,        1);
    CostmapPublisher_       = nodehandle_.advertise<sensor_msgs::Image>         (pubCostmap_,       1);
    RoadPublisher_          = nodehandle_.advertise<sensor_msgs::PointCloud2>   (pubRoad_,          1);
    ObjectPublisher_        = nodehandle_.advertise<sensor_msgs::PointCloud2>   (pubObject_,        1);
    OccupPublisher_         = nodehandle_.advertise<nav_msgs::OccupancyGrid>    (pubOccupancy_,     1);

    subPlane_               = nodehandle_.subscribe(strPlaneTopic_,  1, &Center_map::planeCallback, this);
   subFrotObject_          = nodehandle_.subscribe(strFrotObject_,  1, &Center_map::frotObCb,      this);
}

bool Center_map::readParameters()
{
    nodehandle_.param("sub_plane",              strPlaneTopic_,     string("/plane"));
    nodehandle_.param("sub_back_object",        strBackObject_,     string("/backobject"));
    nodehandle_.param("sub_frot_object",        strFrotObject_,     string("/obstacle_detection/cluster"));

    nodehandle_.param("pub_raw",                pubRawMap_,         string("center_raw"));
    nodehandle_.param("pub_costmap",            pubCostmap_,        string("costmap"));
    nodehandle_.param("pub_road",               pubRoad_,           string("center_road"));
    nodehandle_.param("pub_object",             pubObject_,         string("center_object"));
    nodehandle_.param("pub_occupancy",          pubOccupancy_,      string("occupancy"));
    nodehandle_.param("track_point_frame_id",   trackPointFrameId_, string("vehicle"));

    nodehandle_.param("map_frame_id",           mapFrameId_,        string("vehicle"));
    nodehandle_.param("sensor_frame_id",        sensorFrameId_,     string("velodyne32"));
    nodehandle_.param("robot_base_frame_id",    robotBaseFrameId_,  string("world"));

    nodehandle_.param("track_point_x", trackPoint_.x(), double(0.0));
    nodehandle_.param("track_point_y", trackPoint_.y(), double(0.0));
    nodehandle_.param("track_point_z", trackPoint_.z(), double(0.0));


    nodehandle_.param("map_length", mapLength_, double(30));
    nodehandle_.param("map_resolution", mapResolution_, double(0.4));

    nodehandle_.param("filter_radius", filter_radius_, double(0.2));
    nodehandle_.param("block_circle", block_circle_, double(1.5));

    nodehandle_.param("min_variance", minVariance_, pow(0.003, 2));
    nodehandle_.param("max_variance", maxVariance_, pow(0.03, 2));
    nodehandle_.param("mahalanobis_distance_threshold", mahalanobisDistanceThreshold_, 2.5);
    nodehandle_.param("obstacleHeight",obstacleHeight_,double(-2));

}

void Center_map::cfgCallback(const Parameter cfg)
{
  cfg_ = cfg;
}

void Center_map::output()
{
    grid_map_msgs::GridMap map_msg;
    GridMapRosConverter::toMessage(rawMap_, map_msg);
    map_msg.info.header.frame_id = "vehicle";
    map_msg.info.header.stamp = stamp_;
    MapRawPublisher_.publish(map_msg);

    sensor_msgs::PointCloud2 pointcloud;
    grid_map::GridMapRosConverter::toPointCloud(rawMap_, "elevation", pointcloud);
    pointcloud.header = map_msg.info.header;
    RoadPublisher_.publish(pointcloud);


    rawMap_.clear("road");
    for (GridMapIterator it(rawMap_); !it.isPastEnd(); ++it)
    {
        grid_map::Position currentPosition;
        rawMap_.getPosition(*it, currentPosition);

        rawMap_.at("road", *it) = rawMap_.at("elevation", *it);

        if (rawMap_.at("front_object", *it) > 0.2)
            rawMap_.at("road", *it) = rawMap_.at("front_object", *it);
    }

    if (bridge_flag_)
    {
        Index submapStartIndex(0, (15-2)/0.4);
        Index submapBufferSize(8/0.4, 4/0.4);

        for (grid_map::SubmapIterator iterator(rawMap_, submapStartIndex, submapBufferSize);
            !iterator.isPastEnd(); ++iterator)
        {
          rawMap_.at("road", *iterator) = -1.7;
        }
    }

    grid_map::GridMapRosConverter::toPointCloud(rawMap_, "road", pointcloud);
    pointcloud.header = map_msg.info.header;
    ObjectPublisher_.publish(pointcloud);

    nav_msgs::OccupancyGrid occupancyGrid;
    grid_map::GridMapRosConverter::toOccupancyGrid(rawMap_, "road",
                                                   cfg_.occupancy_min,
                                                   cfg_.occupancy_max,
                                                   occupancyGrid);
    occupancyGrid.header = map_msg.info.header;
    OccupPublisher_.publish(occupancyGrid);


    sensor_msgs::Image outimg;
    grid_map::GridMapRosConverter::toImage(rawMap_,"elevation",
                                           sensor_msgs::image_encodings::MONO8,
                                           cfg_.lower,
                                           cfg_.upper,
                                           outimg);
    outimg.header = map_msg.info.header;
    CostmapPublisher_.publish(outimg);
}

void Center_map::planeCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // stop ros::time
    boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
    stamp_ = cloud_msg->header.stamp;

    //---------Convert sensor_msgs::pc2 to pcl::pc2----------------------------//
    pcl::PCLPointCloud2 pcl_PP2;
    pcl_conversions::toPCL(*cloud_msg, pcl_PP2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_PP2, *origin_cloud);
    pcl_ros::transformPointCloud("vehicle", *origin_cloud, *origin_cloud, transformListener_);

    // Update map localization
    updateMapLocation();
    // Caculate the transformation
    cacuTransform();
    // Caculate the variance
    cacuVariance(origin_cloud);
    // Add point cloud to elevation map.
    addPointcloud(origin_cloud);
    // Filter the costmap
    filterElevationMap();

}

void Center_map::backObCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // stop ros::time
    boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

    //---------Convert sensor_msgs::pc2 to pcl::pc2----------------------------//
    pcl::PCLPointCloud2 pcl_PP2;
    pcl_conversions::toPCL(*cloud_msg, pcl_PP2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_PP2, *origin_cloud);
    pcl_ros::transformPointCloud("vehicle", *origin_cloud, *origin_cloud, transformListener_);

    // Add point cloud to back object map.
//    addBackObject(origin_cloud);
}

void Center_map::frotObCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{

    //ROS_INFO("clustered");
    // stop ros::time
    boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

    //---------Convert sensor_msgs::pc2 to pcl::pc2----------------------------//
    pcl::PCLPointCloud2 pcl_PP2;
    pcl_conversions::toPCL(*cloud_msg, pcl_PP2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_PP2, *origin_cloud);
    pcl_ros::transformPointCloud("vehicle", *origin_cloud, *origin_cloud, transformListener_);

    // Add point cloud to back object map.
    addFrotObject(origin_cloud);
}

bool Center_map::addFrotObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
//    rawMap_.clear("front_object");
    for (GridMapIterator it(rawMap_); !it.isPastEnd(); ++it)
    {
        grid_map::Position currentPosition;
        rawMap_.getPosition(*it, currentPosition);

        rawMap_.at("front_object", *it) = 0;
    }


    for (unsigned int i = 0; i < cloud->size(); ++i) {
      auto& point = cloud->points[i];

      Index index;
      grid_map::Position position(point.x, point.y);
      if (!rawMap_.getIndex(position, index)) continue;

      for (CircleIterator circleIt(rawMap_, position, cfg_.radius);
           !circleIt.isPastEnd(); ++circleIt)
      {
          grid_map::Position currentPositionInCircle;
          rawMap_.getPosition(*circleIt, currentPositionInCircle);

          // Compute weighted mean based on Euclidean distance.
          double weight = pow(position[0] - currentPositionInCircle[0], 2) + pow(position[1] - currentPositionInCircle[1], 2);
          float value = cfg_.weight/sqrt(2*M_PI*pow(cfg_.sigma,2))*exp(-weight/(2*pow(cfg_.sigma,2)));


          grid_map::Index index;
          !rawMap_.getIndex(currentPositionInCircle, index);

          rawMap_.at("front_object", *circleIt) = max(rawMap_.at("front_object", *circleIt) , value);


      }
    }
}

bool Center_map::addPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if (cloud->size() != measurementVariance_.size()) {
      ROS_ERROR("ElevationMap::add: Size of point cloud (%i) and variances (%i) do not agree.",
                (int) cloud->size(), (int) measurementVariance_.size());
      return false;
    }

    for (unsigned int i = 0; i < cloud->size(); ++i) {
      auto& point = cloud->points[i];

      Index index;
      grid_map::Position position(point.x, point.y);
      if (!rawMap_.getIndex(position, index)) continue; // Skip this point if it does not lie within the elevation map.

      auto& elevation = rawMap_.at("elevation", index);
      auto& variance  = rawMap_.at("variance",  index);
      const float& pointVariance = measurementVariance_(i);

      double height = point.z;

      if (point.r < 100) height = 2;

      if (!rawMap_.isValid(index))
      {
        // No prior information in elevation map, use measurement.
        elevation               = height;
        variance                = pointVariance;
        continue;
      }

      double mahalanobisDistance = sqrt(pow(height - elevation, 2) / variance);

      if (mahalanobisDistance < mahalanobisDistanceThreshold_)
      {
        // Fuse measurement with elevation map data.
        elevation               = (variance * height + pointVariance * elevation) / (variance + pointVariance);
        variance                = (pointVariance * variance) / (pointVariance + variance);
        continue;
      }

      // Add noise to cells which have ignored lower values,
      // such that outliers and moving objects are removed.
      variance += multiHeightNoise_;

      variance = variance < minVariance_ ?
                 minVariance_ : (variance > maxVariance_ ?
                 std::numeric_limits<float>::infinity() : variance);
    }

    if (cloud->points[0].b >= 100)
    {
        bridge_flag_ = 1;
        Index submapStartIndex(0, (15-2)/0.4);
        Index submapBufferSize(8/0.4, 4/0.4);

        for (grid_map::SubmapIterator iterator(rawMap_, submapStartIndex, submapBufferSize);
            !iterator.isPastEnd(); ++iterator) {
          rawMap_.at("elevation", *iterator) = -1.7;
      }
    }
    else
    {
        bridge_flag_ = 0;
    }
}


// TODO consider the shifting problem
bool Center_map::updateMapLocation()
{
    ROS_DEBUG ("Elevation map is checked for relocalization");

    geometry_msgs::PointStamped trackPoint;
    trackPoint.header.frame_id = trackPointFrameId_;
    trackPoint.header.stamp = ros::Time(0);

    kindr_ros::convertToRosGeometryMsg(trackPoint_, trackPoint.point);
    geometry_msgs::PointStamped trackPointTransformed;

    try{
        transformListener_.transformPoint(rawMap_.getFrameId(), trackPoint, trackPointTransformed);
    } catch (TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    grid_map::Position position (trackPointTransformed.point.x, trackPointTransformed.point.y);
    rawMap_.move(position);
}

bool Center_map::filterElevationMap()
{
    for (GridMapIterator it(rawMap_); !it.isPastEnd(); ++it)
    {
        grid_map::Position currentPosition;
        rawMap_.getPosition(*it, currentPosition);

        double mean   = 0.0;
        double sumOfWeights = 0.0;

        // Compute weighted mean
        for (CircleIterator circleIt(rawMap_, currentPosition, filter_radius_);
             !circleIt.isPastEnd(); ++circleIt)
        {
            if (!rawMap_.isValid(*circleIt, "elevation")) continue;
            grid_map::Position currentPositionInCircle;
            rawMap_.getPosition(*circleIt, currentPositionInCircle);

            // Compute weighted mean based on Euclidean distance.
            double distance = (currentPosition- currentPositionInCircle).norm();
            double weight = pow(filter_radius_ - distance, 2);
            mean += weight * rawMap_.at("elevation", *circleIt);
            sumOfWeights += weight;
        }

        rawMap_.at("elevation", *it) = mean / sumOfWeights;
    }
}

bool Center_map::cacuTransform()
{
    try {
      transformListener_.waitForTransform(sensorFrameId_, mapFrameId_, stamp_, ros::Duration(1.0));

      // "velodyne64" to "vehicle"
      tf::StampedTransform transformTf;
      transformListener_.lookupTransform(mapFrameId_, sensorFrameId_, ros::Time(0), transformTf);
      poseTFToEigen(transformTf, transformationSensorToMap_);

      // "velodyne64" to "world"
      transformListener_.lookupTransform(robotBaseFrameId_, sensorFrameId_, ros::Time(0), transformTf);  // TODO Why wrong direction?
      Eigen::Affine3d transform;
      poseTFToEigen(transformTf, transform);
      rotationBaseToSensor_.setMatrix(transform.rotation().matrix());
      translationBaseToSensorInBaseFrame_.toImplementation() = transform.translation();

      // "world" to "vehicle"
      transformListener_.lookupTransform(mapFrameId_, robotBaseFrameId_, ros::Time(0), transformTf);  // TODO Why wrong direction?
      poseTFToEigen(transformTf, transform);
      rotationMapToBase_.setMatrix(transform.rotation().matrix());
      translationMapToBaseInMapFrame_.toImplementation() = transform.translation();

      return true;
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      return false;
    }
}

bool Center_map::cacuVariance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Todo ignore the robotPoseCovariance by now
    robotPoseCovariance_.setZero();

    measurementVariance_.resize(cloud->size());

    // Projection vector (P).
    const Eigen::RowVector3f projectionVector = Eigen::RowVector3f::UnitZ();

    // Sensor Jacobian (J_s).
    const Eigen::RowVector3f sensorJacobian = projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();

    // Robot rotation covariance matrix (Sigma_q).
    Eigen::Matrix3f rotationVariance = robotPoseCovariance_.bottomRightCorner(3, 3).cast<float>();

    // Preparations for robot rotation Jacobian (J_q) to minimize computation for every point in point cloud.
    const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
    const Eigen::RowVector3f P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
    const Eigen::Matrix3f C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
    const Eigen::Matrix3f B_r_BS_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));

    for (unsigned int i = 0; i < cloud->size(); ++i)
    {
        // For every point in point cloud.

        // Preparation.
        auto& point = cloud->points[i];
        Eigen::Vector3f pointVector(point.x, point.y, point.z); // S_r_SP
        float heightVariance = 0.0; // sigma_p

        // Measurement distance.
        float measurementDistance = pointVector.norm();

        // Compute sensor covariance matrix (Sigma_S) with sensor model.
        float varianceNormal = 1.0;
        float varianceLateral = 1.0;
        Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
        sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

        // Robot rotation Jacobian (J_q).
        const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
        Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

        // Measurement variance for map (error propagation law).
        heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
        heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();

        // Copy to list.
        measurementVariance_(i) = heightVariance;
    }

    return true;
}
