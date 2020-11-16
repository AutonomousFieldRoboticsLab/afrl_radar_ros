#include <PI_Radar/Radar.h>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>
#include <ros/ros.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
using namespace std;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "radar_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::LaserScan> ("scan",10);
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

  scan.ranges.resize(121, NAN);
  scan.intensities.resize(121, NAN);

  while (ros::ok()) {
    mwRadar.Read(mwr_data,10);
    std::fill(scan.ranges.begin(), scan.ranges.end(), NAN);
    std::fill(scan.intensities.begin(), scan.intensities.end(), NAN);
    
    // cout << "size: " << mwr_data.size() << endl;
    for (int i=0; i<mwr_data.size(); ++i) {
      MWR_Data current = mwr_data[i];
      // TODO: this will overwrite data if the angles are too close together
      int index = 120 - (round(current.Azimuth) + 60);
      // TODO: figure out when this is true
      if (index < 0 || 120 < index) continue;
      
      // cout << "range[" << i << "] = "
      //   << "{ range = " << current.Range
      //   << ", azimuth = " << current.Azimuth
      //   << ", power = " << current.Power
      //   << ", index = " << index
      //   << " }" << endl;
      
      scan.ranges[index] = current.Range;
      scan.intensities[index] = current.Power;
    }
    scan.header.stamp = ros::Time::now();

    pub.publish(scan);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
