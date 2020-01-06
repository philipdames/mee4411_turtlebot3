/*
  Copyright (C) 2013 Nathan Michael

  This file is part of laser_simulator a small laser_simulator for ROS.

  mesh80211s is free software: you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
  details.

  You should have received a copy of the GNU General Public License along with
  this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_simulator/PoseStampedNamedArray.h>
#include <LaserSimulator.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class LaserSimulatorWrapper {
private:
  LaserSimulator sim;
  ros::Publisher pub;
  ros::Subscriber grid_sub, agg_sub;
  sensor_msgs::LaserScan msg;
  boost::mutex sim_mutex;
  std::string map_frame_id;
  tf::TransformListener tf_listener;
  double depth = 0;
  bool map_set = false;

public:
  LaserSimulatorWrapper(ros::NodeHandle &n) : tf_listener(n)
  {
    //ros::Subscriber odom_sub = n.subscribe("odom", 1, handle_odometry);
    grid_sub = n.subscribe("map", 1, &LaserSimulatorWrapper::handle_occupancy_grid, this);
    agg_sub = n.subscribe("pose_array", 1, &LaserSimulatorWrapper::handle_pose_array, this);

    pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);

    n.param("depth", depth, 0.5);

    geometry_msgs::Pose offset;
    n.param("offset/x", offset.position.x, 0.0);
    n.param("offset/y", offset.position.y, 0.0);
    n.param("offset/z", offset.position.z, 0.0);
    double roll, pitch, yaw;
    n.param("offset/roll", roll, 0.0);
    n.param("offset/pitch", pitch, 0.0);
    n.param("offset/yaw", yaw, 0.0);

    tf::Quaternion quat;
    quat.setEuler(yaw, pitch, roll);
    quat.normalize();

    offset.orientation.x = quat.x();
    offset.orientation.y = quat.y();
    offset.orientation.z = quat.z();
    offset.orientation.w = quat.w();

    sim.SetLaserOffset(offset);

    double noise_sd;
    n.param("noise_sd", noise_sd, 0.0);
    if (noise_sd < 0.0)
      {
        ROS_ERROR("%s: noise cannot be negative",
                  ros::this_node::getName().c_str());
        //return -1;
      }
    sim.SetLaserNoiseStdDev(noise_sd);

    if (sim.LoadLaserModel(n) != 0)
      {
        ROS_ERROR("%s: failed to load laser model",
                  ros::this_node::getName().c_str());
        //return -1;
      }

    if (sim.LoadDynamicModels(n) != 0)
      {
        ROS_ERROR("%s: failed to load dynamic models",
                  ros::this_node::getName().c_str());
        //return -1;
      }

    ros::Rate idle(1);
    while (n.ok())
      {
        if (map_set)
          break;
        else
          ROS_DEBUG("%s: waiting for map",
                    ros::this_node::getName().c_str());

        ros::spinOnce();
        idle.sleep();
      }

    std::string frame_id;
    n.param("frame_id", frame_id, std::string("laser"));
    msg.header.frame_id = frame_id;
    msg.angle_min = sim.GetMinimumAngle();
    msg.angle_max = sim.GetMaximumAngle();
    msg.angle_increment = sim.GetAngleIncrement();
    msg.range_min = sim.GetMinimumRange();
    msg.range_max = sim.GetMaximumRange();
    msg.ranges.resize((msg.angle_max - msg.angle_min)
                         /msg.angle_increment + 1);
    msg.intensities.resize(0);
    msg.scan_time = sim.GetScanTime();
    msg.time_increment = msg.scan_time / sim.GetScanCount();
  }

private:
  void handle_odometry(const nav_msgs::Odometry::ConstPtr& odom)
  {
    boost::mutex::scoped_lock(sim_mutex);
    msg.header.stamp = odom->header.stamp;
    msg.header.stamp.nsec += 1000;
    sim.SetPose(odom->pose.pose);
    sim.SetFrameID(odom->child_frame_id);
  }

  void handle_occupancy_grid(const nav_msgs::OccupancyGrid::ConstPtr& grid)
  {
    boost::mutex::scoped_lock(sim_mutex);
    map_frame_id = grid->header.frame_id;
    sim.LoadOccupancyGrid(*grid, depth);
    map_set = true;
  }

  void handle_pose_array(const laser_simulator::PoseStampedNamedArray::ConstPtr& pose)
  {
    boost::mutex::scoped_lock(sim_mutex);
    sim.UpdatePoseArray(*pose);
  }

public:
  void publish(const ros::TimerEvent&) {
    boost::mutex::scoped_lock(sim_mutex);
    msg.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped p, pm;
    // Create pose at origin of laser frame
    p.header.stamp = msg.header.stamp;
    p.header.frame_id = msg.header.frame_id;
    p.pose.orientation.w = 1.0;
    // Transform to map frame to get laser pose
    if (tf_listener.waitForTransform(p.header.frame_id, map_frame_id, p.header.stamp, ros::Duration(sim.GetScanTime())))
      {
        tf_listener.transformPose(map_frame_id, p, pm);
        sim.SetPose(pm.pose);
        // Get scan and publish
        sim.GetScan(msg.ranges);
        pub.publish(msg);
      }
    else
      ROS_WARN("No scan published, transform from %s to %s does not exist", map_frame_id.c_str(), p.header.frame_id.c_str());
  }

  double GetScanTime()
  {
    return sim.GetScanTime();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_simulator");
  ros::NodeHandle n("~");

  LaserSimulatorWrapper ls(n);

  ros::Timer timer = n.createTimer(ros::Duration(ls.GetScanTime()), &LaserSimulatorWrapper::publish, &ls);

  ros::spin();

  return 0;
}
