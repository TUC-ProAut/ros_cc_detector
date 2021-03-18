#include "ros/ros.h"

// message types
#include "costmap_converter/ObstacleArrayMsg.h"
#include "costmap_converter/ObstacleMsg.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
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
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <math.h>
#include <string.h>
#include <vector>

using namespace std;

/*****************************global variables***********************************/
double dist_threshold, dist_tolerance, min_occlusion;
string scan_topic_, base_frame_, output_topic_, output_frame_,
    pointcloud_topic_, cmd_vel_topic_, laserscan_topics_;;
int counter = 0;

/********************************class definition********************************/
class Detector {
public:
  Detector(ros::NodeHandle *nodehandle);

  ros::NodeHandle nh_;
  ros::Subscriber sub_pcl, sub_cmd_vel;
  vector<ros::Subscriber> scan_subscribers;
  ros::Publisher pub, pub_vis;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener;
  tf::StampedTransform transform;

  vector<string> input_topics;
  vector<bool> clouds_modified;

  vector<pcl::PointCloud<pcl::PointXYZ> > cc_clouds;

  double theta_vel;

  // ros message for critical corners
  costmap_converter::ObstacleArrayMsg critical_corners;

protected:
  void scanCallback(const sensor_msgs::LaserScanConstPtr &msg, string topic);
  void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg);
  void laserscan_topic_parser();
};

/********************************constructor************************************/
Detector::Detector(ros::NodeHandle *nodehandle) : nh_(*nodehandle) {
  ROS_INFO_STREAM(scan_topic_.substr(0, scan_topic_.find(" ")).c_str());
  // subscribe up to two different laser scanner 
  sub_cmd_vel = nh_.subscribe(cmd_vel_topic_, 10, &Detector::cmdVelCallback, this);
  pub = nh_.advertise<costmap_converter::ObstacleArrayMsg>(
      output_topic_.c_str(), 10);
  pub_vis = nh_.advertise<sensor_msgs::PointCloud2>("/cc_vis", 1);

  this->laserscan_topic_parser();

}

/********************************functions************************************/
void Detector::laserscan_topic_parser()
{
  // parser is mainly based on the following repo: https://github.com/iralabdisco/ira_laser_tools

	// LaserScan topics to subscribe
	ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);

  istringstream iss(laserscan_topics_);
	vector<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string> >(tokens));
	vector<string> tmp_input_topics;
	for(int i=0;i<tokens.size();++i)
	{
        for(int j=0;j<topics.size();++j)
		{
			if( (tokens[i].compare(topics[j].name) == 0) && (topics[j].datatype.compare("sensor_msgs/LaserScan") == 0) )
			{
				tmp_input_topics.push_back(topics[j].name);
			}
		}
	}

	sort(tmp_input_topics.begin(),tmp_input_topics.end());
	vector<string>::iterator last = unique(tmp_input_topics.begin(), tmp_input_topics.end());
	tmp_input_topics.erase(last, tmp_input_topics.end());

	// Do not re-subscribe if the topics are the same
	if( (tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(),tmp_input_topics.end(),input_topics.begin()))
	{

		// Unsubscribe from previous topics
		for(int i=0; i<scan_subscribers.size(); ++i)
			scan_subscribers[i].shutdown();

		input_topics = tmp_input_topics;
		if(input_topics.size() > 0)
		{
      scan_subscribers.resize(input_topics.size());
			clouds_modified.resize(input_topics.size());
			cc_clouds.resize(input_topics.size());
      ROS_INFO("Subscribing to topics\t%ld", scan_subscribers.size());
			for(int i=0; i<input_topics.size(); ++i)
			{
        scan_subscribers[i] = nh_.subscribe<sensor_msgs::LaserScan> (input_topics[i].c_str(), 1, boost::bind(&Detector::scanCallback,this, _1, input_topics[i]));
				clouds_modified[i] = false;
				cout << input_topics[i] << " ";
			}
		}
		else
      ROS_INFO("Not subscribed to any topic.");
	}
}

void Detector::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg) {
  tf2::Quaternion q;
  this->theta_vel = atan2(msg->linear.y+msg->linear.x*sin(msg->angular.z),msg->linear.x);
  this->theta_vel -= 2*M_PI * floor(this->theta_vel*(1/2*M_PI));  
}

void Detector::scanCallback(const sensor_msgs::LaserScanConstPtr &msg, std::string topic) {

  float x1, x2, y1, y2, inbetween_dist, prev_dist, dist1, dist2, x_temp, y_temp;
  float occlusion = 0;
  bool negativ_jump = 0;
  tf::StampedTransform transform;
  pcl::PointCloud<pcl::PointXYZ> cc_cloud;
  tf2::Quaternion q;

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

  // save modification state of current scan
  for(int j=0; j<input_topics.size(); ++j)
  {
    if(topic.compare(input_topics[j]) == 0)
    {
      cc_clouds[j].clear();
      // ------- actually processing steps --------- //
      for (int i = 1; i < cloud->points.size(); i++) {

        x1 = cloud->points[i - 1].x;
        x2 = cloud->points[i].x;
        y1 = cloud->points[i - 1].y;
        y2 = cloud->points[i].y;

        // measure of the lengths of each laser beam and calculate the difference
        inbetween_dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
        dist1 = sqrt(pow(x1, 2) + pow(y1, 2));
        dist2 = sqrt(pow(x2, 2) + pow(y2, 2));

        //------------ detect positiv jumps and evaluate these ----------------//

        // check if there is a positive jump between the points and enough occlusion
        // by previous points
        if ((dist2 - dist1) > dist_threshold && occlusion > min_occlusion) {
          // use point transformed to output frame
          float x = cloud_output->points[i - 1].x;
          float y = cloud_output->points[i - 1].y;
          
          // check if point is not behind the velocity vector 
          float theta_sens = atan2(cloud_output->points[i].y+this->transform.getOrigin().y(),cloud_output->points[i].x+this->transform.getOrigin().x());
          float theta_diff = this->theta_vel-theta_sens;
          // wrap to [-pi,pi]
          theta_diff = abs(atan2(sin(theta_diff),cos(theta_diff)));

          if(theta_diff<1.6)
          {
            cc_clouds[j].push_back(pcl::PointXYZ(x, y, 0)); 
          }
          
        }

        //------------ accumulate occlusion ----------------//

        // count points with small distance tolerance (like from a shelve)
        if (abs(inbetween_dist) < dist_tolerance) // increments occlusion
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
          negativ_jump = 0;

          // check if point is not behind the velocity vector 
          float theta_sens = atan2(cloud_output->points[i].y+this->transform.getOrigin().y(),cloud_output->points[i].x+this->transform.getOrigin().x());
          float theta_diff=this->theta_vel-theta_sens;
          // wrap to [-pi,pi]
          theta_diff = abs(atan2(sin(theta_diff),cos(theta_diff)));

          if(theta_diff<1.6)
          {
            cc_clouds[j].push_back(pcl::PointXYZ(x_temp, y_temp, 0));
          }
          
        }

      clouds_modified[j] = true;
    }
  }

  // Count how many scans we have
  int totalClouds = 0;
  for(int i=0; i<clouds_modified.size(); ++i)
    if(clouds_modified[i])
      ++totalClouds;

  // Go ahead only if all subscribed scans have arrived
  if(totalClouds == clouds_modified.size())
  {
    pcl::PointCloud<pcl::PointXYZ> cc_cloud;
  
    // clear the cc obstacle message
    costmap_converter::ObstacleArrayMsg critical_corners_temp;
    this->critical_corners = critical_corners_temp;
    this->critical_corners.header.frame_id = output_frame_;
    this->critical_corners.header.stamp = ros::Time::now();

    for(int i=0; i<clouds_modified.size(); ++i)
    {
      for(int n=0; n<cc_clouds[i].points.size();n++)
      {
        cc_cloud.push_back(cc_clouds[i].points[n]);

        // write cc from clouds to obstacle message
        costmap_converter::ObstacleMsg corner;
        corner.header.frame_id = output_frame_;
        corner.header.stamp = ros::Time::now();
        corner.radius = 0;
        geometry_msgs::Point32 corner_point;
        corner_point.x = cc_clouds[i].points[n].x;
        corner_point.y = cc_clouds[i].points[n].y;
        corner.polygon.points.push_back(corner_point);

        // add corner to obstacle array
        this->critical_corners.obstacles.push_back(corner);
      }

      clouds_modified[i] = false;
    }

    // visualize critical corners as point cloud (not obstacles)
    sensor_msgs::PointCloud2 cloud_vis;
    pcl::toROSMsg(cc_cloud, cloud_vis);
    cloud_vis.header.frame_id = output_frame_;
    cloud_vis.header.stamp = ros::Time::now();
    pub_vis.publish(cloud_vis);

  }

}
}

/********************************main************************************/
int main(int argc, char **argv) {
  // Set up ROS.
  ros::init(argc, argv, "detect_critical_corners");

  ros::NodeHandle nh("~");

  // params
  nh.param<std::string>("scan_topics", laserscan_topics_, "");
  nh.param("cmd_vel_topic", cmd_vel_topic_, string("/cmd_vel"));
  nh.param("pointcloud_topic", pointcloud_topic_, string("pointcloud"));
  nh.param("base_frame", base_frame_, string("base_link"));
  nh.param("output_frame", output_frame_, string("odom"));
  nh.param("output_topic", output_topic_, string("/critical_corners"));
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
