#include <PI_Radar/Radar.h>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <radar_msgs/RadarDetection.h>
#include <radar_msgs/RadarDetectionArray.h>
#include <math.h>
#include <inttypes.h>
using namespace std;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "radar_node");
  ros::NodeHandle nh;
  ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan> ("scan",10);
  ros::Publisher radar_pub = nh.advertise<radar_msgs::RadarDetectionArray> ("radar",10);
  ros::Rate loop_rate(10);

  MWRadar mwRadar;
  vector<MWR_Data> mwr_data;

  while (mwr_data.size() == 0 && ros::ok()) {
      ros::Duration(0.1).sleep();
      mwRadar.Read(mwr_data,10);
  }

  sensor_msgs::LaserScan scan;
  scan.header.frame_id = "laser_frame";
  scan.angle_min = -M_PI / 3.0; // radians
  scan.angle_max = M_PI / 3.0; // radians
  scan.range_min = 0.0; // meters
  scan.range_max = 100.0; // meters
  scan.angle_increment = M_PI / 180.0; // radians

  radar_msgs::RadarDetectionArray detections;

  scan.ranges.resize(121, NAN);
  scan.intensities.resize(121, NAN);

  while (ros::ok()) {
    mwRadar.Read(mwr_data,10);
    std::fill(scan.ranges.begin(), scan.ranges.end(), NAN);
    std::fill(scan.intensities.begin(), scan.intensities.end(), NAN);
    detections.detections.clear();

    for (int i=0; i<mwr_data.size(); ++i) {
      MWR_Data current = mwr_data[i];
      radar_msgs::RadarDetection radar_msg;
      // TODO: this will overwrite data if the angles are too close together
      int index = 120 - (round(current.Azimuth) + 60);
      // TODO: figure out when this is true
      if (index < 0 || 120 < index) continue;

      // record scan message
      scan.ranges[index] = current.Range;
      scan.intensities[index] = current.Power;

      // record radar message
      // 1. record position, converting polar to cartesian coordinates
      // TODO: this is inaccurate because of rounding
      radar_msg.position.x = current.Range * cos(index);
      radar_msg.position.y = current.Range * sin(index);
      // 2. record velocity, converting polar to cartesian coordinates
      radar_msg.velocity.x = current.RadialVelocity * cos(index);
      radar_msg.velocity.y = current.RadialVelocity * sin(index);
      // TODO: not sure if these are actually correct
      radar_msg.amplitude = current.Power;
      radar_msg.detection_id = current.index;

      detections.detections.push_back(radar_msg);
    }

    scan.header.stamp = detections.header.stamp = ros::Time::now();
    detections.header.frame_id = "radar";

    laser_pub.publish(scan);
    radar_pub.publish(detections);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
