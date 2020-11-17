#include "ros/ros.h"

// message types
#include "costmap_converter/ObstacleArrayMsg.h"
#include "costmap_converter/ObstacleMsg.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <laser_geometry/laser_geometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <math.h>
#include <string.h>
#include <vector>

/*****************************global variables***********************************/
double dist_threshold, dist_tolerance, min_occlusion;
std::string scan_topic_, base_frame_, output_topic_, output_frame_,
    pointcloud_topic_;
int counter = 0;

/********************************class definition********************************/
class Detector {
public:
  Detector(ros::NodeHandle *nodehandle);

  ros::NodeHandle nh_;
  ros::Subscriber sub_pcl, sub_scan1, sub_scan2;
  ros::Publisher pub, pub_vis;

  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener;
  tf::StampedTransform transform;

  // ros message for critical corners
  costmap_converter::ObstacleArrayMsg critical_corners;

protected:
  void pclCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void scanCallback(const sensor_msgs::LaserScanConstPtr &msg);
};

/********************************constructor************************************/
Detector::Detector(ros::NodeHandle *nodehandle) : nh_(*nodehandle) {
  ROS_INFO_STREAM(scan_topic_.substr(0, scan_topic_.find(" ")).c_str());
  // subscribe up to two different laser scanner 
  sub_scan1 =
      nh_.subscribe(scan_topic_.substr(0, scan_topic_.find(" ")).c_str(), 10,
                    &Detector::scanCallback, this);
  sub_scan2 = nh_.subscribe(
      scan_topic_.substr(scan_topic_.find(" ") + 1, scan_topic_.size()).c_str(),
      10, &Detector::scanCallback, this);
  pub = nh_.advertise<costmap_converter::ObstacleArrayMsg>(
      output_topic_.c_str(), 10);
  pub_vis = nh_.advertise<sensor_msgs::PointCloud2>("/cc_vis", 1);
}

/********************************functions************************************/
void Detector::scanCallback(const sensor_msgs::LaserScanConstPtr &msg) {

  float x1, x2, y1, y2, inbetween_dist, prev_dist, dist1, dist2, x_temp, y_temp;
  float occlusion = 0;
  bool negativ_jump = 0;
  tf::StampedTransform transform;
  pcl::PointCloud<pcl::PointXYZ> cc_cloud;

  // transform
  try {
    listener.lookupTransform(output_frame_, msg->header.frame_id, ros::Time(),
                             this->transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }

  // convert ranges to 2D points
  sensor_msgs::PointCloud2 cloud_ros;
  this->projector_.projectLaser(*msg, cloud_ros);

  // transform point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_ros, *cloud);

  pcl_ros::transformPointCloud(*cloud, *cloud_output, this->transform);

  // decide if critical corners have to be cleared (use the latest 5 scans)
  if (counter < 5) {
    counter++;
  } else {
    counter = 0;
    costmap_converter::ObstacleArrayMsg critical_corners_temp;
    this->critical_corners = critical_corners_temp;
  }

  this->critical_corners.header.frame_id = output_frame_;
  this->critical_corners.header.stamp = ros::Time::now();

  for (int i = 1; i < cloud->points.size(); i++) {
    costmap_converter::ObstacleMsg corner;
    corner.header.frame_id = output_frame_;
    corner.header.stamp = ros::Time::now();
    corner.radius = 0;

    x1 = cloud->points[i - 1].x;
    x2 = cloud->points[i].x;
    y1 = cloud->points[i - 1].y;
    y2 = cloud->points[i].y;

    // measure of the lengths of each laser beam and calculate the difference
    inbetween_dist = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    dist1 = std::sqrt(std::pow(x1, 2) + std::pow(y1, 2));
    dist2 = std::sqrt(std::pow(x2, 2) + std::pow(y2, 2));

    //------------ detect positiv jumps and evaluate these ----------------//

    // check if there is a positive jump between the points and enough occlusion
    // by previous points
    if ((dist2 - dist1) > dist_threshold && occlusion > min_occlusion) {
      geometry_msgs::Point32 corner_point;
      // use point transformed to output frame
      float x = cloud_output->points[i - 1].x;
      float y = cloud_output->points[i - 1].y;
      corner_point.x = x;
      corner_point.y = y;
      corner.polygon.points.push_back(corner_point);
      cc_cloud.push_back(pcl::PointXYZ(x, y, 0));

      // add corner to obstacle array
      this->critical_corners.obstacles.push_back(corner);
    }

    //------------ accumulate occlusion ----------------//

    // count points with small distance tolerance (like from a shelve)
    if (std::abs(inbetween_dist) < dist_tolerance) // increments occlusion
                                                   // distance; if distance is
                                                   // small --> point are
                                                   // probably from a line
    {
      occlusion += inbetween_dist;
    } else {
      occlusion = 0;
      negativ_jump = 0;
    }

    //------------ detect negative jumps and evaluate these ----------------//

    // check if there is a negativ jump between the points and enough occlusion
    // by subsequent points
    if ((dist1 - dist2) > dist_threshold) {
      negativ_jump = 1;
      x_temp = cloud_output->points[i].x;
      y_temp = cloud_output->points[i].y;
    }
    // if negativ jump appears and the subsequent points have a min occlusion,
    // add temp point to critical corners
    if (negativ_jump == 1 && occlusion > min_occlusion) {
      geometry_msgs::Point32 corner_point;
      // use point transformed to output frame
      corner_point.x = x_temp;
      corner_point.y = y_temp;
      corner.polygon.points.push_back(corner_point);
      cc_cloud.push_back(pcl::PointXYZ(x_temp, y_temp, 0));

      negativ_jump = 0;

      // add corner to obstacle array
      this->critical_corners.obstacles.push_back(corner);
    }
  }

  // visualize critical corners as point cloud (not obstacles)
  sensor_msgs::PointCloud2 cloud_vis;
  pcl::toROSMsg(cc_cloud, cloud_vis);
  cloud_vis.header.frame_id = output_frame_;
  cloud_vis.header.stamp = ros::Time::now();
  pub_vis.publish(cloud_vis);
}

/********************************main************************************/
int main(int argc, char **argv) {
  // Set up ROS.
  ros::init(argc, argv, "detect_critical_corners");

  ros::NodeHandle nh("~");

  // params
  nh.param("scan_topic", scan_topic_, std::string("scan"));
  nh.param("pointcloud_topic", pointcloud_topic_, std::string("pointcloud"));
  nh.param("base_frame", base_frame_, std::string("base_link"));
  nh.param("output_frame", output_frame_, std::string("odom"));
  nh.param("output_topic", output_topic_, std::string("/critical_corners"));
  nh.param("dist_threshold", dist_threshold, 1.5);
  nh.param("dist_tolerance", dist_tolerance, 0.1);
  nh.param("min_occlusion", min_occlusion, 0.5);

  Detector detector(&nh);

  // Tell ROS how fast to run this node.
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    detector.pub.publish(detector.critical_corners);
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}