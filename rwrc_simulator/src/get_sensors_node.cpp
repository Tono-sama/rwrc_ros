#include "get_sensors_node.hpp"

// グローバル変数宣言
_raw_topic raw_topic;
_sensor_data sensor_data;
_pointcloud pointcloud;
ros::Publisher odom_pub;

void Init(){
// TODO: グローバル変数初期化
}

int main(int argc, char **argv){

	ros::init(argc, argv, "get_sensors_node");

	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	Init();

	// publisher
	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
	// subscriber
	ros::Subscriber laserscan_sub = n.subscribe("scan", 1, LaserScan_callback);
	ros::Subscriber imu_sub = n.subscribe("imu", 1, Imu_callback);
	ros::Subscriber jointstate_sub = n.subscribe("joint_state", 1, JointState_callback);
	// timer
	ros::Timer main_loop = n.createTimer(ros::Duration(0.1), &Loop);

	// マルチスレッド処理
	ros::AsyncSpinner spinner(0);  //spinを処理するスレッド数を引数に渡す(0はcpuのコア数)
	spinner.start();

	ros::waitForShutdown();

	return 0;
}