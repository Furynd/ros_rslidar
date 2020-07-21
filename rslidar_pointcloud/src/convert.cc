/*
 *  Copyright (C) 2018-2020 Robosense Authors
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Robosense 3D LIDAR packets to PointCloud2.

*/
#include "convert.h"
#include <pcl_conversions/pcl_conversions.h>

namespace rslidar_pointcloud
{
std::string model;

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) : data_(new rslidar_rawdata::RawData())
{
  data_->loadConfigFile(node, private_nh);  // load lidar parameters
  private_nh.param("model", model, std::string("RS16"));

  // advertise output point cloud (before subscribing to input data)
  std::string output_points_topic;
  private_nh.param("output_points_topic", output_points_topic, std::string("/lidar/point_cloud"));
  output_ = node.advertise<sensor_msgs::PointCloud2>(output_points_topic, 10);
  ranges_ = node.advertise<rslidar_msgs::ThreePointData>(std::string("/lidar/rawdist"), 100);
  nearestPub = node.advertise<geometry_msgs::Point>("/lidar/object",100);
  char topic[15];
  for (int i = 0; i < 3; i++)
    {
        sprintf(topic, "/lidar/marker%d", i);
        pos_pub[i] = node.advertise<visualization_msgs::Marker>(topic, 10);
    }

  srv_ = boost::make_shared<dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig> >(private_nh);
  dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig>::CallbackType f;
  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);

  // subscribe to rslidarScan packets
  std::string input_packets_topic;
  private_nh.param("input_packets_topic", input_packets_topic, std::string("rslidar_packets"));
  rslidar_scan_ = node.subscribe(input_packets_topic, 10, &Convert::processScan, (Convert*)this,
                                 ros::TransportHints().tcpNoDelay(true));
}

void Convert::callback(rslidar_pointcloud::CloudNodeConfig& config, uint32_t level)
{
//  ROS_INFO("[cloud][convert] Reconfigure Request");
  // config_.time_offset = config.time_offset;
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const rslidar_msgs::rslidarScan::ConstPtr& scanMsg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->clear();
  if (model == "RS16")
  {
    outPoints->height = 16;
    outPoints->width = 24 * (int)scanMsg->packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }
  else if (model == "RS32" || model == "RSBPEARL" || model == "RSBPEARL_MINI")
  {
    outPoints->height = 32;
    outPoints->width = 12 * (int)scanMsg->packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }

  // process each packet provided by the driver

  data_->block_num = 0;
  float _front_min = 100.0;
  float _right_min = 100.0;
  float _left_min = 100.0;
  float positions[9] = {100,100,100,100,100,100,100,100,100};
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
  {
    data_->unpack(scanMsg->packets[i], outPoints, &_right_min, &_left_min, &_front_min, positions);

  }
  // printf("%.4f %.4f %.4f\n",_front_min, _left_min, _right_min);
  sensor_msgs::PointCloud2 outMsg;
  pcl::toROSMsg(*outPoints, outMsg);

  output_.publish(outMsg);

  rslidar_msgs::ThreePointData out;
  out.header = outMsg.header;
  out.front_dist = _front_min;
  out.left_dist = _left_min;
  out.right_dist = _right_min;
  out.frontPos.x = positions[0];
  out.frontPos.y = positions[1];
  out.frontPos.z = positions[2];
  out.leftPos.x = positions[3];
  out.leftPos.y = positions[4];
  out.leftPos.z = positions[5];
  out.rightPos.x = positions[6];
  out.rightPos.y = positions[7];
  out.rightPos.z = positions[8];

  ranges_.publish(out);
  if(_front_min < 100){
    geometry_msgs::Point object;
    object.x = out.frontPos.x;
    object.y = out.frontPos.y;
    object.z = out.frontPos.z;
    nearestPub.publish(object);
  }

  publishMarker(pos_pub[0], positions[0],positions[1],positions[2],0);
  publishMarker(pos_pub[1], positions[3],positions[4],positions[5],1);
  publishMarker(pos_pub[2], positions[6],positions[7],positions[8],2);
}

int Convert::publishMarker(ros::Publisher pubs, float x, float y, float z, int c)
{

    uint32_t shape = visualization_msgs::Marker::SPHERE;
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/rslidar";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "/rslidar";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x; //harusnya per seribu
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0; ///360*4; //satu putaran 1 atau 2?
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    switch (c)
    {
    case 0:
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        break;

    case 1:
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        break;

    case 2:
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        break;

    case 3:
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        break;

    case 4:
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        break;

    default:
        break;
    }
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    pubs.publish(marker);

    return 0;
}
}  // namespace rslidar_pointcloud
