#include <PI_Radar/Radar.h>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
using namespace std;

int main(int argc, char *argv[]) {

  
  ros::init(argc, argv, "radar_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::LaserScan> ("scan",10);

  MWRadar mwRadar;
  vector<MWR_Data> mwr_data;

  while(mwr_data.size() == 0){
      ros::Duration(0.1).sleep();
      mwRadar.Read(mwr_data,10);
  }

  sensor_msgs::LaserScan scan;
  scan.header.frame_id = "laser_frame";
  scan.angle_min = -M_PI / 3.0;
  scan.angle_max = M_PI / 3.0;
  scan.range_min = 0.0;
  scan.range_max = 20.0;
  scan.angle_increment = M_PI / 180.0;

  ros::Rate loop_rate(20);

  while (ros::ok()) {


    mwRadar.Read(mwr_data,10);
    scan.ranges.resize(mwr_data.size());
    scan.intensities.resize(mwr_data.size());
    for(int i=0;i<mwr_data.size();++i){

      scan.header.stamp = ros::Time::now();
      scan.ranges[i] = mwr_data[i].Range;

    }

  pub.publish(scan);

  ros::spinOnce();
  loop_rate.sleep();

  }
  return 0;
}
