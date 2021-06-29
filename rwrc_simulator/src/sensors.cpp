#include "get_sensors_node.hpp"

void Get_sensors(){
// TODO: センサ生データからオドメトリ計算用の値を取得
}

void Calc_odom(){
// TODO: odom計算
// TODO: tf出力(点群座標変換で必要なので先に計算)
}

void Trans_pointcloud(){
// TODO: 点群座標変換
}

// callback関数
void LaserScan_callback(const sensor_msgs::LaserScan::ConstPtr &laserscan_msg){
	raw_topic.laser_scan = *laserscan_msg;
}

void Imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg){
	raw_topic.imu = *imu_msg;
}

void JointState_callback(const sensor_msgs::JointState::ConstPtr &jointstate_msg){
	raw_topic.joint_state = *jointstate_msg;
}

void Loop(const ros::TimerEvent&){
	Get_sensors();
	Calc_odom();
	Trans_pointcloud();
}

