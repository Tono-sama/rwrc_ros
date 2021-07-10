#include "get_sensors_node.hpp"

// グローバル変数宣言
_raw_topic raw_topic;
_sensor_data sensor_data;
nav_msgs::Odometry odom_msg;
tf2_msgs::TFMessage odom_to_robot_tf_msg;
_pointcloud pointcloud;
tf::TransformListener *listener;
// tf::TransformListener listener;
laser_geometry::LaserProjection *LaserScanToPointCloud;
ros::Publisher odom_pub;
ros::Publisher odom_to_robot_tf_pub;
ros::Publisher odom_point_cloud_pub;

void Init(){
// グローバル変数初期化
	sensor_data.n_o=0.0;sensor_data.o_o=0.0;sensor_data.d_o=0.0;
	sensor_data.n_l_rad=0.0;sensor_data.o_l_rad=0.0;sensor_data.d_r_rad=0.0;
	sensor_data.n_r_rad=0.0;sensor_data.o_r_rad=0.0;sensor_data.d_r_rad=0.0;
	sensor_data.d_l_m=0.0;sensor_data.d_r_m=0.0;
	sensor_data.d_dis=0.0;sensor_data.dis=0.0;
	sensor_data.n_odom.x=0.0;sensor_data.n_odom.y=0.0;sensor_data.n_odom.o=0.0;
	sensor_data.o_odom.x=0.0;sensor_data.o_odom.y=0.0;sensor_data.o_odom.o=0.0;
	// TODO:ROS topicも初期化？
	odom_msg.header.frame_id = "odom";
	odom_msg.child_frame_id = "base_link";
	LaserScanToPointCloud = new laser_geometry::LaserProjection();
	listener = new tf::TransformListener();
}

int main(int argc, char **argv){

	ros::init(argc, argv, "get_sensors_node");

	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	Init();

	// publisher
	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
	odom_to_robot_tf_pub = n.advertise<tf2_msgs::TFMessage>("/tf", 1);
	odom_point_cloud_pub = n.advertise<sensor_msgs::PointCloud>("/trans_point_cloud", 1);
	// subscriber
	ros::Subscriber laserscan_sub = n.subscribe("/scan", 1, LaserScan_callback);
	ros::Subscriber imu_sub = n.subscribe("/imu", 1, Imu_callback);
	ros::Subscriber jointstate_sub = n.subscribe("/simple_icart_middle/joint_states", 1, JointState_callback);
	// timer
	ros::Timer main_loop = n.createTimer(ros::Duration(0.1), &Loop);

	// マルチスレッド処理
	ros::AsyncSpinner spinner(0);  //spinを処理するスレッド数を引数に渡す(0はcpuのコア数)
	spinner.start();

	ros::waitForShutdown();

	return 0;
}