#ifndef GET_SENSORS_NODE_HPP
#define GET_SENSORS_NODE_HPP

// extern.h
// C,C++
#include <stdio.h>
// ROS
#include "ros/ros.h"
// センサ
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
// オドメトリ
#include <nav_msgs/Odometry.h>
// tf
#include <tf2_msgs/TFMessage.h>

// ロボット座標用構造体
struct _xyo{
	double x,y,o;
};

// topicの生データ(subscribeした最新の値を格納)
struct _raw_topic{
	sensor_msgs::LaserScan laser_scan;
	sensor_msgs::Imu imu;
	sensor_msgs::JointState joint_state;
};
extern _raw_topic raw_topic;

// センサデータ(オドメトリ計算用)構造体
struct _sensor_data{
	double n_l,o_l,d_l; // 左のエンコーダ値
	double n_r,o_r,d_r; // 右のエンコーダ値
	double n_o,o_o,d_o; // ヨー角
	_xyo n_odom,o_odom; // オドメトリ
};
extern _sensor_data sensor_data;

// 座標変換後点群データ
struct _pointcloud{
	sensor_msgs::LaserScan robot; // ロボット基準
	sensor_msgs::LaserScan current_map; //現在地図座標基準
};
extern _pointcloud pointcloud;

// subscriber
extern ros::Subscriber laserscan_sub;
extern ros::Subscriber imu_sub;
extern ros::Subscriber jointstate_sub;
// publisher
extern ros::Publisher odom_pub;



// include.h

// sensors.cpp
// func
extern void Get_sensors();
extern void Calc_odom();
extern void Trans_pointcloud();
// callback関数
extern void LaserScan_callback(const sensor_msgs::LaserScan::ConstPtr &laserscan_msg);
extern void Imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg);
extern void JointState_callback(const sensor_msgs::JointState::ConstPtr &jointstate_msg);
extern void Loop(const ros::TimerEvent&);

#endif